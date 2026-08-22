# 本仓库特有的约束

写给在这个 fork 上工作的人和 AI agent。这里**只记录本 fork 与上游 ArduPilot 不同、且踩过坑的地方**。上游通用的 AI 协作规范（代码风格、提交信息、PR 流程等）在 `master` 分支的同名文件里，需要时去那边查。

---

## 1. 编译

```bash
./waf configure --board EFT_CAAC
./waf copter
```

只能在**仓库根目录**跑，`build/` 是产物目录，里面没有 `waf`。

如果报"权限不够"，说明你在的分支上 `waf` 的执行位是丢的——它自 `b008e59941`（2026-05-09 加 EFT_CAAC hwdef 那次）起被记成 `100644`，上游一直是 `100755`。`dev-algo` 已由 `45a30e73ad` 修复，`ardupilot-ubuntu` 尚未。临时绕过用 `python3 ./waf`，根治是 `chmod +x waf` 并提交。

Flash 余量很紧，目前只剩约 **16.8 KB**（1687107 / 1703923 已用）。加代码前先看一眼 BUILD SUMMARY 的 `Free Flash`。

## 2. 改 MAVLink 消息定义是两步操作

**这是本仓库最容易踩的坑，改错了不报任何错。**

上游 ArduPilot 不跟踪 `libraries/GCS_MAVLink/include/`，那是 mavgen 的构建产物目录。本仓库把 448 个生成的头文件**入库**在那里，而 C 语言引号形式的 `#include "include/mavlink/v2.0/..."` 优先搜索当前文件所在目录，所以：

> 编译器读的永远是**源码树里入库的那份**，mavgen 每次写进 `build/` 的那份**没有任何 TU 会 include**。

后果是：只改 `modules/mavlink/message_definitions/` 下的 XML 然后编译，**改动会被静默丢弃**，固件里根本没有新消息。2026-08-05 加进 `eft.xml` 的 msgid 519/520/521（`UOM_ARM_STATUS` / `UOM_FC_STATUS` / `UOM_OPERATOR_ID`）就是这样漏了三周，直到 `7921e9dbb9` 才补进固件。

正确流程：

```bash
# 1. 改完 XML，重新生成
./waf clean && ./waf copter

# 2. 把产物同步回源码树
cp -r build/EFT_CAAC/libraries/GCS_MAVLink/include/mavlink/v2.0/. \
      libraries/GCS_MAVLink/include/mavlink/v2.0/

# 3. 必须再 clean 一次，见下
./waf clean && ./waf copter

# 4. 校验，必须输出 0
diff -rq build/EFT_CAAC/libraries/GCS_MAVLink/include/mavlink/v2.0 \
         libraries/GCS_MAVLink/include/mavlink/v2.0 | wc -l
```

**第 3 步的 clean 不能省。** 增量编译不会重新编译依赖这些入库头文件的 TU——实测同步完头文件直接 `./waf copter`，产出的固件与同步前**逐字节相同**，会让人误以为改动生效了。只有 clean 之后才真正吃进去。

同步会顺带改掉所有 dialect 的 `MAVLINK_BUILD_DATE` 和 `MAVLINK_*_XML_HASH`，几十个文件的 diff 是重新生成的固有噪声，不是异常。

## 3. 不要删除 xyfyf/pymavlink 仓库

定制的 pymavlink 改动只存在于 <https://github.com/xyfyf/pymavlink> 分支 `custom-messages`：

| commit | 内容 |
| --- | --- |
| `f130827f` | 按通道支持 EF 包头发送与解析 |
| `723581df` | msgid 516 帧格式覆盖的魔数和 CRC 字段 |

而 `xyfyf/mavlink` 的 `.gitmodules` 把 pymavlink 指向官方 `ArduPilot/pymavlink.git`。这样能正常工作，是因为 GitHub 的 fork network 共享对象存储，官方 URL 可以取到 fork 里的 sha——**这是隐式依赖**。

一旦 `xyfyf/pymavlink` 被删除，对象可能被 GC，所有人的克隆会同时断链，而且报错完全看不出根因：`Tools/ardupilotwaf/git_submodule.py:69` 对状态为 `+` 的子模块跑 `git merge-base`，遇到不存在的对象 exit 128，异常穿透成 `WafError: Build failed`。

想让依赖显式化，应把 `xyfyf/mavlink` 的 `.gitmodules` 改成 `url = ../pymavlink.git`（相对地址，自动跟随 owner）。

## 4. 帧格式覆盖默认是开启的

`MAV_TX_MAGIC` 默认值 0 = 板级 EF 掩码模式，`EFT_CAAC/hwdef.dat` 规定 **USB(SERIAL0) 和 LINK 遥测(SERIAL6/UART8) 出厂就发 `0xEF` 包头**，不是标准 MAVLink2 的 `0xFD`。

所以任何用**官方** pymavlink 的工具（MAVProxy、`sim_vehicle.py`、autotest、日志解析脚本）都解不出飞控的遥测。仓库里 44 处 `import pymavlink`，`Tools/autotest/sim_vehicle.py:43` 把 `modules/mavlink` 加进 `sys.path`，用的就是子模块里那份定制版——别把它换成 pip 装的版本。

调试时可以看 `mav_tx_override_hits` 计数器确认覆盖逻辑是否真的生效。

## 5. 子模块

`git submodule update --init --recursive` 可以正常跑通（`8bf9f2e035` 清掉了 `mcp-servers/` 下三个没有 `.gitmodules` 条目的 gitlink，那是 `00c9d6c88b` 的误提交，之前会让这条命令直接 fatal）。

**注意 `ardupilot-ubuntu` 上尚未修复**，那三个 gitlink 还在，在该分支上这条命令依然会 fatal，需要加 `-- modules` 限定路径绕过。

如果以后再遇到"某子模块路径在 .gitmodules 中未找到 url"，检查是不是又有内嵌 git 仓库被 `git add .` 扫进索引了。

另：`mcp-servers/ros-mcp/` 不是子模块，是整份第三方源码树入库（207 个文件、91 MB），与固件编译无关，`waf` 不碰它。

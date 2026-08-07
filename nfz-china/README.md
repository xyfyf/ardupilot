# 中国禁飞区离线包（nfz-china）

## 目录

| 路径 | 说明 |
|------|------|
| `sdcard/EFT_nfz/` | **拷到飞控 SD：`EFT/nfz/`**（也可不拷：固件内嵌默认包，开机自动写出） |
| `raw/` | 大疆原始数据 / GeoJSON 预览（再生包用） |
| `tools/rebuild_pack.py` | 从 raw 重新生成 sdcard 包 |
| `SOP.md` | **操作 SOP + 开发思路 + RID 二次开发约定** |

飞控脚本：`ROMFS_custom/scripts/nfz_zone_checker.lua`（或上传到 `EFT/scripts/`）  
默认禁飞区 CSV：`ROMFS_custom/nfz/`（编译进固件 `@ROMFS/nfz/`）

## 快速部署

1. **数据（二选一）**
   - 烧录含 `ROMFS_custom/nfz/` 的固件即可：SD 无 `EFT/nfz/` 时脚本会自动写出；**已有文件不覆盖**
   - 或手动把 `sdcard/EFT_nfz/*` → SD 卡 `EFT/nfz/`
2. 部署 `nfz_zone_checker.lua`
3. 参数：默认 `NFZ_ENABLE=0`（关闭）；要用时设 `NFZ_ENABLE=1`，并建议 `NFZ_ACTION=2`，`NFZ_MARGIN=50`，`NFZ_BASE=1`

细节与 RID 对接见 **[SOP.md](SOP.md)**。

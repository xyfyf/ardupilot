"""Render Mermaid diagrams to PNG/SVG via mermaid.ink.

Each entry in DIAGRAMS = {output_basename: mermaid_source}.
"""
import base64
import urllib.request
import pathlib

DIAGRAMS = {
    "system_architecture": r"""graph TD
    subgraph 载荷与执行端
        A[机载伺服绞盘] -->|实时绳长 L 变化率 L_dot| B(飞控主控单元 MCU)
        C[载荷位姿传感/观测器] -->|低频位姿 摆角 theta 摆速 theta_dot| B
    end

    subgraph 飞控主控单元 MCU 内部逻辑
        B --> D{任务状态机模块}
        D -->|触发 定长巡航模态| E[基准消摆控制器]
        D -->|触发 变长收放模态| F[变绳长自适应控制器]
        D -->|触发 异常降级模态| G[安全接管模块]

        E --> H((控制偏置融合器))
        F --> H
        G -->|清零偏置 绞盘刹车| H
    end

    subgraph 飞行控制底层
        H -->|速度 位置偏置指令| I[底层位置控制器]
        I --> J[姿态 混控输出]
        J --> K[多旋翼动力系统]
    end
""",
    "control_flow": r"""graph TD
    Start([控制周期开始]) --> S1[步骤S1 多源状态数据同步采集]
    S1 --> S2{步骤S2 任务状态识别评估}

    S2 -->|链路超时 或 摆角超限| S3_Degrade[状态C 异常降级模态]
    S2 -->|abs L_dot 大于阈值| S3_Winch[状态B 变长收放模态]
    S2 -->|abs L_dot 近似零 且链路正常| S3_Cruise[状态A 定高巡航模态]

    S3_Degrade --> S4_Safety[执行 偏置清零 绞盘紧急制动]

    S3_Winch --> S4_Adaptive[步骤S3 变绳长自适应控制律解算]
    S4_Adaptive --> S4_a[依据绳长 L 动态更新控制增益 Kp Kv]
    S4_a --> S4_b[引入科里奥利阻尼补偿项 -2 L_dot theta_dot K]
    S4_b --> S5_Offset_Winch[生成综合速度偏置分量 V_offset]

    S3_Cruise --> S4_Static[提取长期静态偏移 过滤风扰静差]
    S4_Static --> S4_c[计算基准速度偏置分量 V_offset]
    S4_c --> S5_Offset_Cruise[生成综合速度偏置分量 V_offset]

    S4_Safety --> S6[步骤S4 底层偏置融合与控制注入]
    S5_Offset_Winch --> S6
    S5_Offset_Cruise --> S6

    S6 --> End([控制周期结束])
""",
}

OUT_DIR = pathlib.Path(__file__).resolve().parent


def encode_for_mermaid_ink(text: str) -> str:
    return base64.urlsafe_b64encode(text.encode("utf-8")).decode("ascii")


def fetch(url: str, dst: pathlib.Path) -> None:
    print(f"[fetch] {url}")
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req, timeout=30) as r:
        data = r.read()
    dst.write_bytes(data)
    print(f"[saved] {dst} ({len(data)} bytes)")


def main() -> None:
    for name, src in DIAGRAMS.items():
        enc = encode_for_mermaid_ink(src)
        png = OUT_DIR / f"{name}.png"
        svg = OUT_DIR / f"{name}.svg"
        try:
            fetch(f"https://mermaid.ink/img/{enc}?type=png&bgColor=white", png)
        except Exception as e:
            print(f"[png-fail {name}] {e}")
        try:
            fetch(f"https://mermaid.ink/svg/{enc}?bgColor=white", svg)
        except Exception as e:
            print(f"[svg-fail {name}] {e}")


if __name__ == "__main__":
    main()

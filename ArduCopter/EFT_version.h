#pragma once

// EFT 软件构型标识（见 .cursor/rules/git-tag-release-notes.mdc）
// 格式: FC{硬件主}.{硬件子}_S{软件主}.{软件子}.{修订}_{版本类别}_{飞机类型}
// 发版时只改 S 后面的 3.0.34，其余 FC2.0 / _R / _T 保持不变
#define EFT_FIRMWARE_VERSION "FC2.0_S3.0.37_R_T"

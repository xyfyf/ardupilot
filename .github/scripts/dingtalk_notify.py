#!/usr/bin/env python3
import os, hmac, hashlib, base64, time, json, re, sys
import urllib.request, urllib.parse

tag     = os.environ["TAG_NAME"]
webhook = os.environ["DINGTALK_WEBHOOK"]
secret  = os.environ.get("DINGTALK_SECRET", "")
body    = os.environ.get("TAG_BODY", "").strip()

if body:
    lines = body.splitlines()
    # 第一行作为标题（通常格式：vX.X.X: 说明）
    title_line = lines[0].strip() if lines else tag
    rest_lines = [l for l in lines[1:] if l.strip()]
    rest = "\n".join(rest_lines).strip()
    # 【xxx】加粗
    rest = re.sub(r'【(.+?)】', r'\n\n**【\1】**', rest)
    md_title = f"🚁 固件发布 {tag}"
    md_text  = (
        f"## 🚁 {title_line}\n\n"
        f"{rest}"
    )
else:
    md_title = f"🚁 固件发布 {tag}"
    md_text  = (
        f"## 🚁 固件发布 {tag}\n\n"
        f"（本次发布未填写说明）"
    )

# 计算钉钉加签
ts = str(int(time.time() * 1000))
if secret:
    str_to_sign = f"{ts}\n{secret}"
    sig  = base64.b64encode(
        hmac.new(secret.encode("utf-8"), str_to_sign.encode("utf-8"), hashlib.sha256).digest()
    ).decode()
    sign = urllib.parse.quote_plus(sig)
    url  = f"{webhook}&timestamp={ts}&sign={sign}"
else:
    url = webhook

payload = json.dumps({
    "msgtype": "markdown",
    "markdown": {"title": md_title, "text": md_text},
    "at": {"isAtAll": False}
}, ensure_ascii=False).encode("utf-8")

print(f"版本：{tag}")
print(f"标题：{md_title}")

req = urllib.request.Request(
    url, data=payload,
    headers={"Content-Type": "application/json; charset=utf-8"}
)
with urllib.request.urlopen(req, timeout=10) as resp:
    result = resp.read().decode()
    print("钉钉返回：", result)
    resp_json = json.loads(result)
    if resp_json.get("errcode", 0) != 0:
        print("发送失败：", resp_json)
        sys.exit(1)
    print("✅ 钉钉消息已发送")

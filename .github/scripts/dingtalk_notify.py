#!/usr/bin/env python3
import os, hmac, hashlib, base64, time, json, re, sys
import urllib.request, urllib.parse

tag     = os.environ["TAG_NAME"]
repo    = os.environ["REPO"]
webhook = os.environ["DINGTALK_WEBHOOK"]
secret  = os.environ.get("DINGTALK_SECRET", "")
body    = os.environ.get("TAG_BODY", "").strip()

repo_url = f"https://github.com/{repo}"
tag_url  = f"{repo_url}/releases/tag/{tag}"

if body:
    lines = body.splitlines()
    title_line = lines[0] if lines else tag
    rest = "\n".join(lines[1:]).strip()
    rest = re.sub(r'【(.+?)】', r'**【\1】**', rest)
    md_text  = f"## 🚁 {title_line}\n\n{rest}\n\n---\n[📦 查看Tag]({tag_url})  |  [📁 仓库]({repo_url})"
    md_title = f"🚁 固件更新 {tag}"
else:
    md_title = f"🚁 固件更新 {tag}"
    md_text  = f"## 🚁 {tag}\n\n（未填写 tag 说明）\n\n[📦 查看Tag]({tag_url})"

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

print(f"发送到: {url[:80]}...")
print(f"标题: {md_title}")

req = urllib.request.Request(url, data=payload,
                             headers={"Content-Type": "application/json; charset=utf-8"})
with urllib.request.urlopen(req, timeout=10) as resp:
    result = resp.read().decode()
    print("钉钉返回：", result)
    resp_json = json.loads(result)
    if resp_json.get("errcode", 0) != 0:
        print("ERROR: 发送失败", resp_json)
        sys.exit(1)
    print("✅ 钉钉消息发送成功")

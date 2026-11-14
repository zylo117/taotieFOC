"""从 PDF 里抠文本：解压所有 FlateDecode 流，取 (…)Tj / [..]TJ 里的字符串，
然后按关键词找 STEP 时序规格。用法：py -X utf8 scripts/pdf_grep_tmp.py <pdf> <关键词...>
"""
import re, sys, zlib

path = sys.argv[1]
keys = [k.upper() for k in sys.argv[2:]] or ['STEP']
raw = open(path, 'rb').read()
out = []
for m in re.finditer(rb'stream\r?\n', raw):
    start = m.end()
    end = raw.find(b'endstream', start)
    if end < 0:
        continue
    blob = raw[start:end]
    try:
        data = zlib.decompress(blob)
    except Exception:
        data = blob          # 未压缩的流直接用
    # 文本操作符里的字符串：(...)Tj 与 [(...)..]TJ
    for tm in re.finditer(rb'\((?:\\.|[^\\()])*\)', data):
        s = tm.group(0)[1:-1]
        s = s.replace(b'\\(', b'(').replace(b'\\)', b')').replace(b'\\\\', b'\\')
        try:
            out.append(s.decode('latin-1'))
        except Exception:
            pass
text = ' '.join(out)
text = re.sub(r'\s+', ' ', text)
print(f'--- 提取字符数 {len(text)} ---')
# 按句子/片段找关键词命中处的前后文
for k in keys:
    for m in re.finditer(re.escape(k), text, re.IGNORECASE):
        a = max(0, m.start() - 140)
        b = min(len(text), m.end() + 200)
        seg = text[a:b]
        if any(ch.isdigit() for ch in seg):
            print(f'[{k}] ...{seg}...')
            print()

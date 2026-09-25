#!/usr/bin/env python3
# Per-frame CUDA API counts and host time, steady window only (after the 10th sensor_update NVTX range).
import sqlite3, sys, collections
db = sqlite3.connect(sys.argv[1])
q = lambda s, *a: db.execute(s, a).fetchall()
S = dict(q("select id, value from StringIds"))
nv = q("select start from NVTX_EVENTS where text='sensor_update' or textId in (select id from StringIds where value='sensor_update') order by start")
t0 = nv[10][0]; t1 = nv[-1][0]
kern = q("select k.start, k.correlationId, coalesce(k.shortName, k.demangledName) from CUPTI_ACTIVITY_KIND_KERNEL k where k.start>=? and k.start<=?", t0, t1)
phys_corr = set(c for s, c, n in kern if 'phys_cam' in S.get(n, ''))
frames = sum(1 for s, c, n in kern if 'cuda_phys_cam_noise_kernel' in S.get(n, ''))
rt = q("select start, end, nameId, globalTid, correlationId from CUPTI_ACTIVITY_KIND_RUNTIME where start>=? and start<=?", t0, t1)
phys_tids = set(g for s, e, n, g, c in rt if c in phys_corr)
cnt = collections.Counter(); dur = collections.Counter(); cnt_all = collections.Counter(); dur_all = collections.Counter()
for s, e, n, g, c in rt:
    name = S[n].split('_v')[0]
    cnt_all[name] += 1; dur_all[name] += (e - s)
    if g in phys_tids:
        cnt[name] += 1; dur[name] += (e - s)
print("window %.1f ms, phys-cam frames %d, phys-cam render thread(s) %d" % ((t1 - t0) / 1e6, frames, len(phys_tids)))
print("%-28s %10s %12s %14s | %10s %12s" % ("API (phys-cam thread)", "count", "per frame", "host ms/frame", "all thr", "all ms/frame"))
for name in sorted(set(cnt_all), key=lambda k: -dur_all[k]):
    if cnt_all[name] == 0: continue
    print("%-28s %10d %12.2f %14.4f | %10d %12.4f" % (name, cnt[name], cnt[name] / frames, dur[name] / 1e6 / frames, cnt_all[name], dur_all[name] / 1e6 / frames))
tot = sum(dur.values()) / 1e6 / frames
print("phys-cam thread total CUDA API host time per frame: %.4f ms" % tot)

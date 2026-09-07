"""Write real_exp/RESULTS.md from the accepted trial directories on disk."""
from __future__ import annotations
import json
from pathlib import Path

ROOT = Path('/home/dhruv/projects_dhruv/namo/robot_control/real_exp')
ARMS = [('model_pure_policy', 'model policy'), ('model_search', 'model search'),
        ('uniform_search', 'uniform search')]
# (catalog, uid, scene, tier, results subpath, arm dir overrides)
ENVS = [
    ('1hop_simple', 'hmax2__easy_020__5f0639e3', 'hmax2/easy_020', 'easy', 'results/real/hmax2__easy_020__5f0639e3', {}),
    ('1hop_simple', '1push__hard_021__bf3e3cdf', '1push/hard_021', 'hard', 'results/real/1push__hard_021__bf3e3cdf',
     {'model_search': 'model_r*', 'uniform_search': 'random_r*'}),
    ('1hop_simple', '2push__env__obstacle_0_movable__0489c6b2', 'v3/hard_loose/rb_00180', 'hard',
     'results/real/2push__env__obstacle_0_movable__0489c6b2', {}),
    ('1hop_multi_int', '2push__env__obstacle_0_movable__febcb94c', 'v2/zig_solo0/rb_00034', 'hard',
     'results/real/2push__env__obstacle_0_movable__febcb94c/variants/original', {}),
    ('1hop_multi_int', '2push__env__obstacle_0_movable__fbdce248', 'v2/dense_solo0/rb_00121', 'medium',
     'results/real/2push__env__obstacle_0_movable__fbdce248', {}),
]


def read_trial(d: Path):
    try:
        summary = json.loads((d / 'summary.json').read_text())
    except FileNotFoundError:
        return None
    def rows(name):
        f = d / name
        if not f.exists():
            return []
        return [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
    pushes, plans = rows('pushes.jsonl'), rows('plans.jsonl')
    cfg = {}
    if (d / 'config.json').exists():
        cfg = json.loads((d / 'config.json').read_text())
    return {
        'name': d.name,
        'outcome': summary.get('outcome'),
        'reason': summary.get('outcome_reason', ''),
        'pushes': len(pushes),
        'stuck': sum(1 for p in pushes if p.get('stuck')),
        'objects': [p.get('object_id') for p in pushes],
        'plans': len(plans),
        'sims': sum(int(p.get('simulations_used') or 0) for p in plans),
        'ms': sum(float(p.get('planning_wall_time_ms') or 0) for p in plans),
        'namo_cpp': ((cfg.get('repositories') or {}).get('namo_cpp') or {}).get('commit', ''),
        'started': cfg.get('started_at_utc', '')[:10],
    }


def arm_trials(root: Path, arm: str, override: str | None):
    # The policy arm's directory is model_pure_policy on the older rooms and
    # model_policy on the two multi-interaction ones; take whichever exists.
    patterns = [override] if override else [f'{arm}/trial*']
    if not override and arm == 'model_pure_policy':
        patterns.append('model_policy/trial*')
    pattern = next((pat for pat in patterns if any(root.glob(pat))), patterns[0])
    return sorted((t for t in (read_trial(p) for p in sorted(root.glob(pattern)) if p.is_dir()) if t),
                  key=lambda t: t['name'])


lines = [
    '# Real-robot results',
    '',
    'Generated from the accepted trial directories under `1hop_simple/results/real/` and',
    '`1hop_multi_int/results/real/`. Runs under any `invalid/` path are excluded and are not',
    'counted anywhere here. [EXPERIMENT_STATUS.md](EXPERIMENT_STATUS.md) is the collection',
    'ledger and says what is still owed; this file reports what the collected trials measured.',
    '',
    'The protocol is five physical trials per arm per environment, `trialN` on explicit seed',
    'N-1. The three arms share one planner and differ only in how the next push is chosen:',
    'model policy takes the ranker arg-max with zero physics rollouts, model search runs',
    'best-first with the ranker as its prior, and uniform search runs the same search with a',
    'uniform prior and no checkpoint. Navigation baselines qualify an environment and do not',
    'count toward the 15.',
    '',
]

total_done = total_trials = 0
summary_rows = []
detail = []

for cat, uid, scene, tier, sub, overrides in ENVS:
    root = ROOT / cat / sub
    per_arm = {}
    for arm, label in ARMS:
        per_arm[arm] = arm_trials(root, arm, overrides.get(arm))
    counts = []
    for arm, label in ARMS:
        ts = per_arm[arm]
        ok = sum(1 for t in ts if t['outcome'] == 'success')
        counts.append(f'{ok}/{len(ts)}' if ts else '0/0')
        total_done += len(ts)
        total_trials += 5
    complete = all(len(per_arm[a]) == 5 for a, _ in ARMS)
    summary_rows.append((cat, uid, scene, tier, counts, complete))

    detail.append(f'## `{uid}`')
    detail.append('')
    detail.append(f'Scene `{scene}`, tier {tier}, catalog `{cat}`.')
    detail.append('')
    for arm, label in ARMS:
        ts = per_arm[arm]
        if not ts:
            detail.append(f'**{label}.** Not collected.')
            detail.append('')
            continue
        ok = sum(1 for t in ts if t['outcome'] == 'success')
        detail.append(f'**{label}.** {ok} of {len(ts)} reached the goal.')
        detail.append('')
        detail.append('| trial | outcome | pushes | stuck | plans | sims | planning |')
        detail.append('| --- | --- | ---: | ---: | ---: | ---: | ---: |')
        for t in ts:
            secs = f"{t['ms'] / 1000:.1f} s" if t['ms'] else 'n/a'
            detail.append(f"| {t['name']} | {t['outcome']} | {t['pushes']} | {t['stuck']} | "
                          f"{t['plans']} | {t['sims']} | {secs} |")
        detail.append('')

lines.append('## Where collection stands')
lines.append('')
lines.append(f'{total_done} of {total_trials} protocol trials are collected across '
             f'{len(ENVS)} environments.')
lines.append('')
lines.append('| environment | scene | tier | model policy | model search | uniform search | complete |')
lines.append('| --- | --- | --- | --- | --- | --- | --- |')
for cat, uid, scene, tier, counts, complete in summary_rows:
    lines.append(f'| `{uid}` | `{scene}` | {tier} | {counts[0]} | {counts[1]} | {counts[2]} | '
                 f'{"yes" if complete else "no"} |')
lines.append('')
lines.append('Counts read successes over trials collected, not over the five the protocol asks for.')
lines.append('')
lines += detail

(ROOT / 'RESULTS.md').write_text('\n'.join(lines).rstrip() + '\n')
print(f'{total_done}/{total_trials} trials collected; wrote', ROOT / 'RESULTS.md')
for cat, uid, scene, tier, counts, complete in summary_rows:
    print(' ', uid, counts, 'complete' if complete else '')

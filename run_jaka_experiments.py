#!/usr/bin/env python3
"""Repeated simulation runs; append raw output and results to jaka_log.txt.

Usage: python3 run_jaka_experiments.py --runs 10 --mode gui
Only Python's standard library is required. Every round restarts this workcell.
Target IDs are visual queue IDs, not Gazebo entity IDs. Counts are log evidence,
not independent physical verification of objects inside the receiving bin.
"""

import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid


WS = Path(__file__).resolve().parent
LOG = WS / 'jaka_log.txt'
STOP = False
ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')


def request_stop(_signum, _frame):
    global STOP
    STOP = True


def write_log(stream, message):
    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    line = f'[{stamp}] {message}'
    stream.write(line + '\n')
    stream.flush()
    print(line, flush=True)


def stop_group(process):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            continue
        # A shell can exit before its ROS children; terminate the group too.
        if sig != signal.SIGKILL:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        break
    process.wait()


def command(stream, script, timeout, env, allow_stop=True):
    write_log(stream, f'执行 {script}，超时限制 {timeout}s')
    start = LOG.stat().st_size
    process = subprocess.Popen(
        ['bash', str(WS / script)], cwd=WS, env=env,
        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
    )
    deadline = time.monotonic() + timeout
    status = None
    while process.poll() is None:
        if allow_stop and STOP:
            status = 'INTERRUPTED'
            break
        if time.monotonic() >= deadline:
            status = 'TIMEOUT'
            break
        time.sleep(0.2)
    if status:
        stop_group(process)
    end = LOG.stat().st_size
    return process.returncode, status, start, end


def parse_result(path, start, end):
    attempts = []
    active = None
    finished = None
    errors = []
    with open(path, 'rb') as source:
        source.seek(start)
        while source.tell() < end:
            line = ANSI.sub('', source.readline().decode('utf-8', 'replace'))
            match = re.search(
                r'\[PICK cycle=(\d+) confirmed=\d+\] id=(\d+) '
                r'position=\(([^)]+)\).*?edge=(True|False) attempt=(\d+)', line)
            if match:
                active = dict(
                    cycle=int(match[1]), target_id=int(match[2]),
                    xyz_base_link=[float(x) for x in match[3].split(',')],
                    edge=match[4] == 'True', attempt=int(match[5]),
                    result='UNKNOWN', yaw_deg=None, reason='',
                )
                attempts.append(active)
            if active:
                yaw = re.search(r'SELECTED GRASP CANDIDATE: yaw=([+\-\d.]+)', line)
                if yaw:
                    active['yaw_deg'] = float(yaw[1])
                if '[GRASP POINT]' in line:
                    active['grasp_point'] = line.split('[GRASP POINT]', 1)[1].strip()
                failed = re.search(r'\[PICK FAILED\] id=(\d+)(.*)', line)
                if failed and int(failed[1]) == active['target_id']:
                    active['result'] = 'FAILED'
                    active['reason'] = failed[2].strip(' :')
                if '[SCENE UPDATE]' in line and 'target_removed=True' in line:
                    active['result'] = 'SUCCESS'
                    active = None
                elif '[SCENE UPDATE]' in line and 'target_removed=False' in line:
                    if active['result'] == 'UNKNOWN':
                        active['result'] = 'FAILED'
                        active['reason'] = '队列未确认放置成功'
                    active = None
            match = re.search(r'DYNAMIC PICK FINISHED \| confirmed placed=(\d+)', line)
            if match:
                finished = int(match[1])
            if 'DYNAMIC PICK FAILED:' in line or 'process has died' in line:
                errors.append(line.strip())
    return attempts, finished, errors


def summarize(attempts, finished, errors, count, code, status):
    successful = {a['target_id'] for a in attempts if a['result'] == 'SUCCESS'}
    latest = {a['target_id']: a for a in attempts}
    failed = [a for key, a in latest.items()
              if key not in successful and a['result'] == 'FAILED']
    unknown = [a for key, a in latest.items()
               if key not in successful and a['result'] == 'UNKNOWN']
    placed = len(successful)
    consistent = finished == placed == count
    outcome = status or ('SUCCESS' if code == 0 and not errors and consistent else 'FAILED')
    return dict(
        outcome=outcome, requested_count=count, confirmed_placed=placed,
        failed_target_count=len(failed), failed_targets=failed,
        failed_attempt_count=sum(a['result'] == 'FAILED' for a in attempts),
        uncertain_targets=unknown,
        unaccounted_count=max(0, count - placed - len(failed)),
        declared_placed=finished, exit_code=code, errors=errors,
        attempts=attempts,
    )


def recommendations(result):
    reasons = ' '.join(a['reason'] for a in result['attempts']) + ' '.join(result['errors'])
    advice = []
    if 'COLLISION' in reasons or any(a['edge'] and a['result'] == 'FAILED' for a in result['attempts']):
        advice.append('检查失败坐标附近的料斗壁、碰撞模型和下降/退出路径；对同一区域复测。')
    if 'SUCTION' in reasons:
        advice.append('核对吸盘 TCP、目标表面高度和吸附距离日志。')
    if 'scene' in reasons.lower() or 'VISION' in reasons:
        advice.append('检查 RGB-D 数据新鲜度、TF 和场景稳定判断。')
    if result['outcome'] == 'TIMEOUT':
        advice.append('检查最后停留阶段及规划耗时，再判断是否调整单轮超时。')
    if result['outcome'] != 'SUCCESS' and not advice:
        advice.append('结合本轮原始日志检查启动、规划及恢复阶段；日志不足时保持未确认状态。')
    return advice or ['继续积累不同随机布局样本，比较失败区域和重试成功率。']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=0, help='轮数，0 表示持续运行至 Ctrl+C')
    parser.add_argument('--count', type=int, choices=range(1, 7), default=6)
    parser.add_argument('--mode', choices=['gui', 'headless'], default='gui')
    parser.add_argument('--timeout', type=float, default=1200, help='每轮抓取超时秒数')
    parser.add_argument('--startup-timeout', type=float, default=300)
    parser.add_argument('--interval', type=float, default=5)
    parser.add_argument('--max-consecutive-failures', type=int, default=5)
    args = parser.parse_args()
    if (args.runs < 0 or min(args.timeout, args.startup_timeout) <= 0
            or args.interval < 0 or args.max_consecutive_failures < 1):
        parser.error('轮数/间隔不能为负，超时和连续失败上限必须为正')
    env = dict(os.environ, AUBO_GRASP_WS=str(WS),
               AUBO_RANDOM_CUBE_COUNT=str(args.count), PYTHONUNBUFFERED='1')
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    # Lock the output itself: no second experiment can overwrite/reset the scene.
    with open(LOG, 'a', encoding='utf-8', buffering=1) as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(2, '已有实验脚本正在运行，未启动第二份。\n')
        session = str(uuid.uuid4())
        write_log(stream, f'实验开始 session={session} options={vars(args)}')
        write_log(stream, '坐标为 base_link 米制坐标；yaw 为吸盘角度。目标 ID 仅在本轮内有效。')
        write_log(stream, '成功依据程序放置确认；未确认数不等于物理剩余数。每轮重启本工作区仿真。')
        failures = 0
        rounds = 0
        successes = 0
        try:
            while not STOP and (args.runs == 0 or rounds < args.runs):
                rounds += 1
                begun = time.monotonic()
                write_log(stream, f'===== 第 {rounds} 轮开始 =====')
                code, status, _, _ = command(
                    stream, f'start_{args.mode}_aubo.sh', args.startup_timeout, env)
                if code == 0 and status is None and not STOP:
                    code, status, start, end = command(stream, 'run_random_pick.sh', args.timeout, env)
                    attempts, finished, errors = parse_result(LOG, start, end)
                else:
                    attempts, finished, errors = [], None, ['仿真启动未完成，未执行抓取']
                result = summarize(attempts, finished, errors, args.count, code, status)
                result.update(session=session, round=rounds,
                              duration_seconds=round(time.monotonic() - begun, 2))
                write_log(stream, 'ROUND_RESULT ' + json.dumps(result, ensure_ascii=False))
                write_log(stream, f"本轮 {result['outcome']}：确认完成 {result['confirmed_placed']} 个；"
                          f"最后尝试失败且未成功 {result['failed_target_count']} 个；"
                          f"失败尝试 {result['failed_attempt_count']} 次；"
                          f"未确认/未尝试 {result['unaccounted_count']} 个。")
                for item in result['failed_targets'] + result['uncertain_targets']:
                    write_log(stream, f"目标 {item['target_id']}：{item['result']}，"
                              f"xyz={item['xyz_base_link']}，yaw={item['yaw_deg']}°，"
                              f"边缘={item['edge']}，原因={item['reason'] or '中断或未获得结果'}")
                for advice in recommendations(result):
                    write_log(stream, '建议（日志推断）：' + advice)
                ok = result['outcome'] == 'SUCCESS'
                successes += int(ok)
                failures = 0 if ok else failures + 1
                clean_code, clean_status, _, _ = command(stream, 'cleanup_aubo_system.sh', 90, env, False)
                if clean_code != 0 or clean_status:
                    write_log(stream, '清理未正常完成，停止实验，避免下一轮状态混杂。')
                    break
                if failures >= args.max_consecutive_failures:
                    write_log(stream, '达到连续失败上限，停止实验；可调整 --max-consecutive-failures。')
                    break
                deadline = time.monotonic() + args.interval
                while not STOP and time.monotonic() < deadline:
                    time.sleep(0.2)
        finally:
            command(stream, 'cleanup_aubo_system.sh', 90, env, False)
            write_log(stream, f'实验结束：共 {rounds} 轮，整轮成功 {successes} 轮；日志 {LOG}')
    return 130 if STOP else (0 if successes == rounds else 1)


if __name__ == '__main__':
    raise SystemExit(main())

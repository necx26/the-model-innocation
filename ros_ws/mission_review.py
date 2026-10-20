"""Produce a review from measured mission phases and navigation records."""
def review_markdown(state, phases):
    total=state.get('total',5)
    passed=(state.get('phase')=='完成' and state.get('placed')==total
            and state.get('elapsed',301)<300)
    lines=['# 本轮任务复盘','',
           f"结果：{'通过' if passed else '未通过'}，{state.get('placed',0)}/{total}；用时 {state.get('elapsed',0):.1f} 秒。",
           f"结束状态：{state.get('status','')}。",'',
           '| 阶段 | 实测秒数 |','| --- | ---: |']
    lines += [f'| {name} | {seconds:.2f} |' for name,seconds in sorted(phases.items(),key=lambda item:-item[1])]
    notes=[]
    elapsed=max(1,state.get('elapsed',1))
    margin=max(0,300-state.get('elapsed',300))
    if passed and margin<30:
        notes.append(f'时间余量只有 {margin:.1f} 秒，低于 30 秒工程目标：还不能视作稳定的限时方案。')
    search=sum(phases.get(name,0) for name in ('搜索隔间','定向识别','等待重试','经验引导复核'))
    if search>elapsed*.3:notes.append('搜索占比超过 30%：优先检查有效相机视野、重复观察点和已检查空区，不先提高最高车速。')
    placed=state.get('placed',0)
    if placed and phases.get('放置',0)/placed>10:notes.append('每件放置及退出平均超过 10 秒：检查终点容差、IK 重试和不必要的底盘朝向调整。')
    metrics=state.get('navigation_metrics',[])
    failed=[m for m in metrics if m.get('outcome') not in ('arrived','region_reachable','cancelled')]
    if failed:notes.append(f'存在 {len(failed)} 次导航异常：查看对应目标、转向量和失败原因后再调整恢复策略。')
    if '冻结' in state.get('status',''):notes.append('检测到导航位置冻结：检查项目 tf2 修复是否加载，禁止继续沿过期位置运动。')
    if 'IK' in state.get('status',''):notes.append('末端不可达：先核对停稳后的工作空间与底盘间距，再修改抓取或放置动作。')
    if metrics:
        lines += ['', '## 导航实测', '',
                  '| 目标 x,y | 秒数 | 行驶米数 | 累计转向弧度 | 结果 |',
                  '| --- | ---: | ---: | ---: | --- |']
        for item in sorted(metrics,key=lambda m:-m.get('seconds',0))[:5]:
            goal=item.get('goal',[0,0])
            lines.append(f"| {goal[0]},{goal[1]} | {item.get('seconds',0):.2f} | {item.get('travelled_m',0):.2f} | {item.get('turned_rad',0):.2f} | {item.get('outcome','')} |")
        lines += ['', '累计转向包含正常转弯；结合轨迹和没有平移进展的事件区分正常绕墙与异常原地转动。']
    results=state.get('result',[])
    if results:
        good=sum(item.get('final_verified',False) for item in results)
        lines += ['', f'结束位置复核：{good}/{len(results)} 件仍在指定放置区。']
    if state.get('grasp_attempts'):
        attempts=state['grasp_attempts'];picked=state.get('picked',0)
        lines += ['', f'抓取核验：{attempts} 次尝试，{picked} 次抬升确认。']
        if attempts>picked:notes.append('出现未通过抬升核验的抓取：检查手腕朝向、夹爪碰角、定位误差与碰撞，不只看最终完成数。')
    if not passed and not notes:notes.append('按结束状态和逐帧记录定位首个失败动作，修复后使用相同场景复测。')
    if passed:notes.append('本轮满足 300 秒与逐件入区检查；还需用不同题目、顺序和布局验证稳定性。')
    lines += ['', '## 下一轮检查', '']+[f'- {note}' for note in notes]
    lines += ['', '本记录来自实际任务，不表示任意未知地图和布局都能保证限时完成。','']
    return '\n'.join(lines)

"""Auditable arithmetic tool for the generator's container-demand question family.

No layout, simulator state, random seed or precomputed answer is read. Unknown
wording is rejected. Each named person's demands are derived from input text.
"""
import math
import re

CHINESE = {'一':1,'二':2,'两':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9,'十':10}


def demand_tool(question):
    line=re.search(r'题目\s*[:：]\s*([^\n]+)',question)
    if line:question=line[1]
    text=question.replace('：',':').replace('，',',').replace('；',';')
    capacity=re.search(r'每[个家只](.+?)有(\d+)',text)
    kinds=re.search(r'有([^;。,]+?)[、,]([^;。,]+?)两种',text)
    if not capacity or not kinds:raise ValueError('算术工具不支持此题型，需更换正式模型适配器')
    size=int(capacity[2]);items=[kinds[1],kinds[2]]
    if size<=0:raise ValueError('容量必须为正')
    people={};steps=[]
    clauses=re.split(r'[;。]',text)
    for clause in clauses:
        if not clause or clause.lstrip().startswith(('题目:每','每','有','设')):continue
        # Names in the supplied generator are followed by discourse words as
        # well as verbs: e.g. ``小勇说和小壮一样`` and ``小康也和小壮一样``.
        names=re.findall(r'小[\u4e00-\u9fff]{1,2}?(?=、|和|跟|都|与|需|想|领|购|订|要|说|也|一样|相同|借阅|借了|借|读|看|买|购买|租用|借用|拿|,|的)',clause)
        if not names:raise ValueError('无法确定题目子句的主体：'+clause)
        counts=[0,0]
        relation=('一样' in clause or '相同' in clause)
        if relation:
            if len(names)!=2 or names[1] not in people:raise ValueError('无法确定引用的需求：'+clause)
            subject=[names[0]];counts=list(people[names[1]])
            replacement=re.search(r'(?:换成|改成|换为|改为)('+ '|'.join(map(re.escape,items))+')',clause)
            if replacement:
                # Preserve the referenced quantity while replacing its item
                # category, as in ``same as Xiao Zhuang, but change to football``.
                amount=sum(counts);counts=[0,0];counts[items.index(replacement[1])]=amount
            for sign,amount,item in re.findall(r'(多|少)(\d+|[一二两三四五六七八九十])(?:个|件|只|条|本|支|瓶|份|盒|袋|杯|箱|块|颗|粒|片)?('+ '|'.join(map(re.escape,items))+')',clause):
                amount=int(amount) if amount.isdigit() else CHINESE[amount]
                counts[items.index(item)]+=amount*(1 if sign=='多' else -1)
        else:
            subject=names
            for i,item in enumerate(items):
                values=re.findall(r'(\d+)(?:个|件|只|条|本|支|瓶|份|盒|袋|杯|箱|块|颗|粒|片)?'+re.escape(item),clause)
                if len(values)>1:raise ValueError('同一子句出现多个相同物品数量，需人工核对：'+clause)
                if values:counts[i]=int(values[0])
            if not any(counts):raise ValueError('无法读取需求数量：'+clause)
            if '双胞胎' in clause or '双倍' in clause:counts=[n*2 for n in counts]
        if any(n<0 for n in counts):raise ValueError('需求出现负数')
        for name in subject:
            if name in people:raise ValueError('人物重复，不能确定累加还是替换：'+name)
            people[name]=list(counts)
        steps.append(dict(clause=clause,subjects=subject,per_person=counts))
    if not people:raise ValueError('未提取到需求')
    totals=[sum(d[i] for d in people.values()) for i in range(2)]
    x,y=[math.ceil(n/size) for n in totals]
    if x+y!=5 or not 1<=x<=4 or not 1<=y<=4:raise ValueError('题意统计结果不满足已知赛题约束x+y=5')
    return dict(items=items,capacity=size,demands=people,steps=steps,totals=totals,x=x,y=y)

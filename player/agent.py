from typing import List
inst = List[dict]


class Agent(object):
    def __init__(self, name, side, **kwargs):
        """必要的初始化"""
        self.name = name
        self.side = side

    def reset(self, **kwargs):
        """重置智能体"""
        pass

    def step(self, **kwargs) -> inst:
        """输入态势信息，返回指令列表"""
        raise NotImplementedError


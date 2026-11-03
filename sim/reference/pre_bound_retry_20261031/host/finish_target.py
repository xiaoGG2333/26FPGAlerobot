"""Observe completion of the firmware's final goal without changing it."""
class CompletionTracker:
    def __init__(self):
        self.reset()
    def reset(self):
        self.goal=None
        self.samples=0
    def observe(self, actual, goal):
        if any(len(v)!=5 or any(type(c) is not int or not 0<=c<=4095 for c in v) for v in (actual,goal)):
            raise ValueError('Invalid completion feedback')
        if self.goal!=goal:
            self.goal=list(goal)
            self.samples=0
        gap=max(abs(g-a) for g,a in zip(goal,actual))
        self.samples=self.samples+1 if gap<=8 else 0
        return self.samples>=3, gap

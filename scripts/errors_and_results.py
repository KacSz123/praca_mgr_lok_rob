class LocalizationResult:
    def __init__(self, pose:list, orientation:float, ksiX:float=None,ksiY:float=None):
        self.pose = pose
        self.age = orientation
        self.ksiX = ksiX
        self.ksiY = ksiY
    def __str__(self) -> str:
        return 'pose: '+str(self.pose)+', orientation: '+str(self.orientation)+', ksiX: '+str(self.ksiX)+', ksiY: ' +str(self.ksiX)

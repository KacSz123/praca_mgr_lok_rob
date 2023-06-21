from typing import Any


class LocalizationResult:
    """ Class to represent result of localization a mobile robot
        using histogram methods or normal distribution method.

        Atributes
        ---------
        pose : list
            a list of current position [x,y]
        orientation : float
            a  orientation value in radians
        xiX : float
            value of computed parameter xi for X axis
        xiY : float
            value of computed parameter xi for Y axis
    """
    def __init__(self, pose:list, orientation:float, xiX:float=None,xiY:float=None):
        """Contructor of instance of class LokalizationResult, 
        xi* default values are None for basic method

        Args:
            pose (list): [x,y] position of robot on XY plane
            orientation (float): Theta orientation of robot in radians
            xiX (float, optional): Calculated value of xi parameter for X axis. None for basic algorithms. Defaults to None.
            xiY (float, optional): Calculated value of xi parameter for X axis. None for basic algorithms. Defaults to None.
        """
        self.pose = pose
        self.orientation = orientation
        self.xiX = xiX
        self.xiY = xiY
    def __str__(self) -> str:
        """ dunder method returns string desciption of this instance of object

        Returns:
            str: desciption of this instance of object
        """
        return 'pose: '+str(self.pose)+', orientation: '+str(self.orientation)+', xiX: '+str(self.xiX)+', xiY: ' +str(self.xiX)
    
    def __getitem__(self, atribute : str):
        """dunder method to get value one of the atribute

        Args:
            atribute (str): key name of atribute

        Raises:
            TypeError: If Arg atribute is not instance of str
            ValueError: If atribute is not in ['pose', 'orientation', 'xiX', 'xiY'] 

        Returns:
            _type_: list or float, depend of which atribute is called
        """
        if type(atribute)!=str:
            raise TypeError('atribute must be a string')
        elif atribute=='pose':
            return self.pose
        elif atribute=='orientation':
            return self.orientation
        elif atribute=='xiX':
            return self.xiX
        elif atribute=='xiY':
            return self.xiY
        else:
            raise ValueError(" atribute must be one of those str:'pose', 'orientation', 'xiX', 'xiY'")
        



###### in progress
class MapErrors(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)
    pass
class HistLocalizationErrors(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)
    pass
class ProbabLocalizationErrors(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)
    pass    

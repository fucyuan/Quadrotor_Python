import numpy as np

def RPYtoRot_ZXY(phi, theta, psi):
    """
    将欧拉角（滚转角、俯仰角、偏航角）转换为从机体坐标系到世界坐标系的旋转矩阵。
    
    参数：
    phi   : 滚转角（绕 x 轴的旋转）
    theta : 俯仰角（绕 y 轴的旋转）
    psi   : 偏航角（绕 z 轴的旋转）
    
    返回值：
    R     : 旋转矩阵，从机体坐标系转换到世界坐标系的 3x3 矩阵。
    """

    # 计算旋转矩阵 R，按照 ZXY 顺序，Z 表示偏航角，X 表示滚转角，Y 表示俯仰角
    R = np.array([
        [np.cos(psi) * np.cos(theta) - np.sin(phi) * np.sin(psi) * np.sin(theta),
         np.cos(theta) * np.sin(psi) + np.cos(psi) * np.sin(phi) * np.sin(theta),
         -np.cos(phi) * np.sin(theta)],
        
        [-np.cos(phi) * np.sin(psi),
         np.cos(phi) * np.cos(psi),
         np.sin(phi)],
        
        [np.cos(psi) * np.sin(theta) + np.cos(theta) * np.sin(phi) * np.sin(psi),
         np.sin(psi) * np.sin(theta) - np.cos(psi) * np.cos(theta) * np.sin(phi),
         np.cos(phi) * np.cos(theta)]
    ])
    
    return R

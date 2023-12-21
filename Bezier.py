#Code modified from https://blog.csdn.net/weixin_39258979/article/details/108083655
import matplotlib.pyplot as plt
import numpy as np
import math

class Bezier:
    # 输入控制点，Points是一个array,num是控制点间的插补个数
    def __init__(self,Points,InterpolationNum):
        self.demension=Points.shape[1]   # 点的维度
        self.order=Points.shape[0]-1     # 贝塞尔阶数=控制点个数-1
        self.num=InterpolationNum        # 相邻控制点的插补个数
        self.pointsNum=Points.shape[0]   # 控制点的个数
        self.Points=Points
        
    # 获取Bezeir所有插补点
    def getBezierPoints(self,method):
        if method==0:
            return self.DigitalAlgo()
        if method==1:
            return self.DeCasteljauAlgo()
    
    # 数值解法
    def DigitalAlgo(self):
        PB=np.zeros((self.pointsNum,self.demension)) # 求和前各项
        pis =[]                                      # 插补点
        for u in np.arange(0,1+1/self.num,1/self.num):
            for i in range(0,self.pointsNum):
                PB[i]=(math.factorial(self.order)/(math.factorial(i)*math.factorial(self.order-i)))*(u**i)*(1-u)**(self.order-i)*self.Points[i]
            pi=sum(PB).tolist()                      #求和得到一个插补点
            pis.append(pi)            
        return np.array(pis)

    # 德卡斯特里奥解法
    #使用该解法输入数组元素需要加.0，如[10,20]要改为[10.0, 20.0]，原因不明
    def DeCasteljauAlgo(self):
        pis =[]                          # 插补点
        for u in np.arange(0,1+1/self.num,1/self.num):
            Att=self.Points
            for i in np.arange(0,self.order):
                for j in np.arange(0,self.order-i):
                    Att[j]=(1.0-u)*Att[j]+u*Att[j+1]
            pis.append(Att[0].tolist())
        return np.array(pis)

class Line:
    def __init__(self,Points,InterpolationNum):
        self.demension=Points.shape[1]    # 点的维数
        self.segmentNum=InterpolationNum-1 # 段数
        self.num=InterpolationNum         # 单段插补(点)数
        self.pointsNum=Points.shape[0]   # 点的个数
        self.Points=Points                # 所有点信息
        
    def getLinePoints(self):
        # 每一段的插补点
        pis=np.array(self.Points[0])
        # i是当前段
        for i in range(0,self.pointsNum-1):
            sp=self.Points[i]
            ep=self.Points[i+1]
            dp=(ep-sp)/(self.segmentNum)# 当前段每个维度最小位移
            for i in range(1,self.num):
                pi=sp+i*dp
                pis=np.vstack((pis,pi))         
        return pis



if __name__ == '__main__':
    # points=np.array([
    #     [1,3,0],
    #     [1.5,1,0],
    #     [4,2,0],
    #     [4,3,4],
    #     [2,3,11],
    #     [5,5,9]
    #     ])

    # points=np.array([
    #     [0.0,0.0],
    #     [1.0,0.0],
    #     [1.0,1.0],
    #     [0.0,1.0],
    #     ])

    #转弯半径
    R = 10
    InterpolationNum=100
    # path = [[10, 10], [11, 11], [12, 12], [13, 13], [14, 14], [15, 15], [16, 16], [17, 17], [18, 18], [19, 19], [20, 20], [21, 21], [22, 22], [23, 22], [24, 22],  
    #         [25, 22], [26, 22], [27, 22], [28, 22], [29, 22], [30, 22], [31, 22], [32, 22], [33, 22], [34, 22], [35, 22], [36, 22], [37, 22], [38, 22], [39, 22], [40, 22], [41, 22], [42, 22], [43, 22], [44, 22], [45, 22], [46, 22], [47, 22], [48, 22], [49, 22], [50, 23], [51, 24], 
    #         [52, 25], [53, 26], [54, 27], [55, 28], [56, 29], [57, 30], [58, 31], [59, 32], [60, 33], [61, 34], [62, 35], [63, 36], [64, 37], [65, 38], [66, 39], [67, 40], [68, 41], [69, 42], [70, 43], [71, 44], [72, 45], [73, 46], [74, 47], [75, 48], [76, 49], [77, 50], [78, 51], 
    #         [79, 52], [80, 53], [81, 54], [82, 55], [83, 56], [84, 57], [85, 58], [86, 58], [87, 58], [88, 58], [89, 58], [90, 58], [91, 58], [92, 59], [93, 60], [94, 61], [95, 62], [96, 63], [97, 64], [98, 65], [99, 66], [100, 67], [100, 68], [100, 69], [100, 70], [100, 71], [100, 72], 
    #         [100, 73], [100, 74], [100, 75], [100, 76], [100, 77], [100, 78], [100, 79], [101, 80], [102, 81], [103, 82], [104, 83], 
    #         [105, 84], [105, 85], [105, 86], [105, 87], [105, 88], [105, 89], [105, 90], [105, 91], [105, 92], [105, 93], [105, 94], [105, 95], [105, 96], [104, 97], [103, 98], [102, 99], [101, 100], [100, 100]]
    path = [[10, 10], [10, 10], [11, 11], [12, 12], [13, 13], [14, 14], [15, 15], [16, 16], [17, 17], [18, 18], [19, 19], [20, 20], [19, 21], [20, 22], [21, 23], [21, 24], [21, 25], [21, 26], [21, 27], [21, 28], 
            [21, 29], [21, 30], [21, 31], [22, 32], [23, 33], [24, 34], [25, 35], [26, 36], [25, 37], [26, 38], [27, 39], [27, 40], [26, 41], [26, 42], [27, 43], [28, 44], [29, 45], [30, 46], [31, 47], [30, 48], 
            [29, 49], [30, 50], [31, 51], [30, 52], [29, 53], [30, 54], [31, 55], [32, 56], [33, 57], [34, 58], [35, 59], [36, 60], [37, 59], [38, 58], [39, 59], [40, 60], [41, 61], [42, 62], [43, 63], [44, 64], 
            [45, 65], [46, 66], [47, 67], [48, 68], [49, 69], [50, 69], [51, 69], [52, 70], [53, 71], [54, 72], [55, 73], [56, 74], [57, 75], [58, 76], [59, 77], [60, 78], [61, 79], [62, 80], [63, 79], [64, 80], 
            [65, 81], [66, 80], [67, 79], [68, 80], [69, 79], [70, 78], [71, 77], [72, 76], [73, 75], [74, 74], [75, 73], [76, 72], [77, 73], [78, 74], [79, 75], [80, 76], [81, 77], [82, 78], [83, 79], [84, 80], 
            [83, 81], [84, 82], [83, 83], [84, 84], [83, 85], [84, 86], [85, 87], [86, 88], [87, 89], [88, 90], [89, 91], [89, 92], [89, 93], [89, 94], [89, 95], [90, 96], [91, 97], [92, 98], [93, 97], [94, 98], 
            [95, 97], [96, 98], [97, 99], [98, 100], [99, 101], [100, 100]]
    
    #分段贝塞尔
    #判断是否变向，如果变向就取前后2R个点做贝塞尔平滑，向前2R遇到变向点就再向前（后）取R个点
    def change_dir(pos0, pos1, pos2):
        vec1 = (pos1[0]-pos0[0], pos1[1]-pos0[1])
        vec2 = (pos2[0]-pos1[0], pos2[1]-pos1[1])
        return vec1 != vec2
    
    #返回方向改变点（及其索引）
    def get_turnings(path):
        turnings = []
        turnings_index = []
        for i in range(1, len(path)-1):
            if change_dir(path[i-1], path[i], path[i+1]):
                turnings_index.append(i)
                turnings.append(path[i])
        return turnings, turnings_index
    
    #返回各段需要平滑的曲线的控制点列表，使用辅助函数
    def control_points(path):
        turnings, turnings_index = get_turnings(path)
        #正在搜索的拐点在拐点列表中的索引
        index = 0
        result = []
        while index != len(turnings_index):
            cp_set, index = control_points_ap(path, turnings_index, [path[turnings_index[index]]], index, False)
            result.append(cp_set)
            index += 1
        return result
    
    #返回每一小段曲线的控制点列表以及当前搜索的拐点的index，使用递归
    def control_points_ap(path, turnings_index, cp_set, index, extending):
        index_in_path = turnings_index[index]

        #边界条件
        #左边界
        if extending == False:
            if index_in_path < R:
                cp_set.insert(0, path[0])
            else:
                cp_set.insert(0, path[index_in_path - R])
        #右边界
        if index_in_path + 2*R > len(path) - 1:
            #最后一个拐点
            if index == len(turnings_index) - 1:
                if index_in_path + R > len(path) - 1:
                    cp_set.append(path[-1])
                else:
                    cp_set.append(path[index_in_path + R])
                return cp_set, index
            #非最后一个拐点
            else:
                cp_set.append(path[turnings_index[index + 1]])
                index += 1
                return control_points_ap(path, turnings_index, cp_set, index, True)
        #非边界最后一个拐点
        if index == len(turnings_index) - 1:
            cp_set.append(path[index_in_path + R])
            return cp_set, index
            
        #在转弯半径内出现第二个拐点就并入这一段曲线并寻找下一个控制点
        if index_in_path + 2*R > turnings_index[index + 1]:
            cp_set.append(path[turnings_index[index + 1]])
            index += 1
            return control_points_ap(path, turnings_index, cp_set, index, True)
        #转弯半径内没有下一个拐点，直接返回
        else:
            cp_set.append(path[index_in_path + R])
            return cp_set, index
            
    cps = control_points(path)
    # print(path)
    #print(cps)
    lines = []
    curves = []
    for cp in cps:
        points = np.array(cp)
        points.astype(float)
        l = Line(points,InterpolationNum)
        lines.append(l.getLinePoints())
        bz = Bezier(points,InterpolationNum)
        curves.append(bz.getBezierPoints(0))
    pl = lines[0]
    matpi = curves[0]
    for i in range(1, len(lines)):
        pl = np.append(pl, lines[i], axis=0)
        matpi = np.append(matpi, curves[i], axis=0)
    # print(pl)
    plt.plot(pl[:,0],pl[:,1],color='k')
    plt.plot(matpi[:,0],matpi[:,1],color='r')
    plt.show()
    #print(points)

    # #3D曲线
    # if points.shape[1]==3:
    #     fig=plt.figure()
    #     ax = fig.gca(projection='3d')
    #     # 标记控制点
    #     for i in range(0,points.shape[0]):
    #         ax.scatter(points[i][0],points[i][1],points[i][2],marker='o',color='r')
    #         ax.text(points[i][0],points[i][1],points[i][2],i,size=12)
    #     # 直线连接控制点
    #     l=Line(points,InterpolationNum)
    #     pl=l.getLinePoints()
    #     ax.plot3D(pl[:,0],pl[:,1],pl[:,2],color='k')
    #     # 贝塞尔曲线连接控制点
    #     bz=Bezier(points,InterpolationNum)
    #     matpi=bz.getBezierPoints(0)
    #     ax.plot3D(matpi[:,0],matpi[:,1],matpi[:,2],color='r')
    #     plt.show()
    
    # points = np.array([[105, 91], [105, 96], [101, 100], [100, 100]])
    # # 2D曲线
    # if points.shape[1]==2:  
    #     # 标记控制点
    #     for i in range(0,points.shape[0]):
    #         plt.scatter(points[i][0],points[i][1],marker='o',color='r')
    #         plt.text(points[i][0],points[i][1],i,size=12)
    #     #直线连接控制点
    #     l=Line(points,InterpolationNum)
    #     pl=l.getLinePoints()
    #     plt.plot(pl[:,0],pl[:,1],color='k')
    #     # 贝塞尔曲线连接控制点
    #     bz=Bezier(points,InterpolationNum)
    #     matpi=bz.getBezierPoints(0)
    #     plt.plot(matpi[:,0],matpi[:,1],color='r')
    #     plt.show()
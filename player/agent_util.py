import pandas as pd
from scipy.spatial.distance import cdist
from env.env_def import *
# import math
from env.env_util import *
import os
import datetime
from pathlib import Path
from itertools import chain

class RedEntityNum:
    A2A = 20
    A2G = 16
    AWACS = 1
    DISTURB = 1
    UAV = 3
    SHIP = 2
    S2A = 0
    RADAR = 1
    AIRPORT = 1
    COMMAND = 0

class BlueEntityNum:
    A2A = 12
    A2G = 8
    AWACS = 1
    DISTURB = 0
    UAV = 0
    SHIP = 1
    S2A = 3
    RADAR = 2
    AIRPORT = 1
    COMMAND = 2

# add by lq
class AirDefenseCoefficient:
    A2A_COEFF = 1.05
    A2G_COEFF = 1.4
    AWACS_COEFF = 1.2
    DISTURB_COEFF = 1.2
    UNAIRTAR_COEFF = 1.1

class AirDefenseMaxChannel:
    SHIP_CHANNEL = 4
    S2A_CHANNEL = 3

class AirDefenseMunSpeed:
    SHIP_BOARD = 1300.0
    SHORE_BOARD = 1300.0

class AirDefenseMunRange:
    SHIP_BOARD = 100000.0
    SHORE_BOARD = 100000.0

class TargetMaxSpeed:
    A2A = 1000.0/3.6
    A2G = 800.0/3.6
    AWACS = 800.0/3.6
    DISTURB = 800.0/3.6
    UNAIRTAR = 800.0/3.6

def lengthofVector(vector):
    return (vector[0]**2+vector[1]**2)**0.5

import math


def polarToCartesian(length,angle):
    y = length*math.sin(angle)
    x = length*math.cos(angle)
    return x, y


def cartesianToPolar(x,y):
    return (x**2+y**2)**0.5, math.atan2(y, x)

# 类型转换
def ListDict2DataFrame(list_dict):
    return pd.DataFrame(list_dict)

# 获取情报数据 数据类型dataframe
def GetQBData(qb_list_dict):
    return ListDict2DataFrame(qb_list_dict)

# 获取己方作战单元数据 数据类型dataframe
def GetOwnUnitsData(units_list_dict):
    return ListDict2DataFrame(units_list_dict)

# 获取敌方弹药数据
def GetEnemyMunData(muns_list_dict):
    return ListDict2DataFrame(muns_list_dict)

def GetAirportData(airport_list_dict):
    return ListDict2DataFrame(airport_list_dict)

def GetTeamsID_ByType(own_teams,type):
    flag= False
    teams_id_list = own_teams[(own_teams.LX == type)]['TMID'].tolist()
    if len(teams_id_list)>0:
        flag = True
    return flag, teams_id_list

def GetPlatformIDListInTeams(own_teams,teamid):
    flag = False
    platform_id_list = own_teams[(own_teams.TMID == teamid)]['PT'].tolist()
    if len(platform_id_list)>0:
        flag = True

    return flag, platform_id_list

# 根据作战单元类型获取己方作战单元ID
def GetOwnUnitsID_ByType(own_units_df, type):
    flag = False
    units_id_list = own_units_df[(own_units_df.LX == type) & (own_units_df.WH == 1)]['ID'].tolist()

    if len(units_id_list)>0:
        flag = True

    return flag, units_id_list

# 根据导弹数据判断我方实体是否被攻击,返回攻击者列表
def JudgeAircraftBeAttacked_ByID(rockets_df, unit_id):
    flag = False
    #print("rockets_df=",rockets_df)
    attacker_id_list = []
    if len(rockets_df)>0:
        attacker_id_list = rockets_df[(rockets_df.N2 == unit_id) & (rockets_df.WH == 1)]['N1'].tolist()

    if len(attacker_id_list)>0:
        flag = True

    return flag,attacker_id_list

# 根据作战单元类型获取敌方作战单元ID
def GetQBUnitsID_ByType(qb_units_df, type, jb_id):
    # jb_id 0表示蓝方 1表示红方
    flag = False
    units_id_list = qb_units_df[(qb_units_df.LX == type) & (qb_units_df.JB == jb_id)]['ID'].tolist()

    if len(units_id_list)>0:
        flag = True

    return flag, units_id_list

# 根据作战单元ID获取其位置 注：返回值记得取列表的第一个元素[0]
def GetUnitPosByID(df, unit_id, dim):
    flag = False

    if dim==2:
        unit_pos = df[df.ID == unit_id][['X','Y']].values.tolist()
    else:
        unit_pos = df[df.ID == unit_id][['X', 'Y', 'Z']].values.tolist()

    if len(unit_pos)>0:
        flag = True

    return flag, unit_pos

# 根据弹药ID获取其位置
def GetMunAngleByID(df, mun_id):
    flag = False

    mun_angle = df[df.ID == mun_id][['FY','FG', 'HX']].values.tolist()

    if len(mun_angle)>0:
        flag = True

    return flag, mun_angle

# 根据作战单元ID获取其状态
def GetUnitStateByID(df, unit_id):
    flag = False
    unit_state = df[df.ID == unit_id]['ST'].values.tolist()

    if len(unit_state)>0:
        flag = True

    return flag, unit_state

# 根据作战单元ID获取其航向
def GetUnitHeadingByID(df, unit_id):
    flag = False
    unit_heading = df[df.ID == unit_id]['HX'].values.tolist()

    if len(unit_heading)>0:
        flag = True

    return flag, unit_heading

# 根据作战单元ID获取其速度
def GetUnitSpeedByID(df, unit_id):
    flag = False
    unit_speed = df[df.ID == unit_id]['SP'].values.tolist()

    if len(unit_speed)>0:
        flag = True

    return flag, unit_speed

# 根据作战单元ID获取其编组ID
def GetUnitTeamIDByID(df, unit_id):
    flag = False
    unit_team_id = df[df.ID == unit_id]['TMID'].values.tolist()

    if len(unit_team_id)>0:
        flag = True

    return flag, unit_team_id

# 根据作战单元ID获取其剩余弹药数量
def GetUnitMunNumByID(df, unit_id):
    flag = False
    unit_mun_num = -1
    unit_mun_info = df[df.ID == unit_id]['WP']

    if len(unit_mun_info)>0:
        unit_mun_num = list(unit_mun_info.tolist()[0].values())[0]
        flag = True

    return flag, unit_mun_num

# 根据作战单元ID列表获取其状态列表
def GetUnitsStateByIDList(df, unit_id_list):
    flag = False
    unit_state_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_state = GetUnitStateByID(df, unit_id)
        if pos_flag:
            unit_state_list.append(unit_state[0])
            pos_flag = True
        else:
            unit_state_list.append(None)
            pos_flag = False

    return flag, unit_state_list

# 根据作战单元ID列表获取其航向列表
def GetUnitsHeadingByIDList(df, unit_id_list):
    flag = False
    unit_heading_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_heading = GetUnitStateByID(df, unit_id)
        if pos_flag:
            unit_heading_list.append(unit_heading[0])
            pos_flag = True
        else:
            unit_heading_list.append(None)
            pos_flag = False

    return flag, unit_heading_list

# 根据作战单元ID列表获取其速度列表
def GetUnitsSpeedByIDList(df, unit_id_list):
    flag = False
    unit_speed_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_speed = GetUnitSpeedByID(df, unit_id)
        if pos_flag:
            unit_speed_list.append(unit_speed[0])
            pos_flag = True
        else:
            unit_speed_list.append(None)
            pos_flag = False

    return flag, unit_speed_list

# 根据作战单元ID列表获取其编队ID列表
def GetUnitsTeamIDByIDList(df, unit_id_list):
    flag = False
    unit_team_id_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_team_id = GetUnitTeamIDByID(df, unit_id)
        if pos_flag:
            unit_team_id_list.append(unit_team_id[0])
            pos_flag = True
        else:
            unit_team_id_list.append(None)
            pos_flag = False

    return flag, unit_team_id_list

# 根据作战单元ID列表获取其剩余弹药数量列表
def GetUnitsMunNumByIDList(df, unit_id_list):
    flag = False
    unit_mun_num_list = []
    #有点bug-wutao
    for unit_id in unit_id_list:
        s_flag, unit_mun_num = GetUnitMunNumByID(df, unit_id)
        if s_flag:
            unit_mun_num_list.append(unit_mun_num)
            flag = True
        else:
            unit_mun_num_list.append(None)
            flag = False

    return flag, unit_mun_num_list

# 根据作战单元ID列表获取其位置列表
def GetUnitsPosByIDList(df, unit_id_list, dim):
    flag = False
    unit_pos_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_pos = GetUnitPosByID(df, unit_id, dim)
        if pos_flag:
            unit_pos_list.append(unit_pos[0])
            flag = True
        else:
            if dim==2:
                unit_pos_list.append([None, None])
                flag = False
            else:
                unit_pos_list.append([None, None, None])
                flag = False

    return flag, unit_pos_list

# 根据作战单元ID获取其完整实体信息
def GetUnitInfoByID(df, unit_id):
    flag = False
    unit_info = df[df.ID == unit_id]

    if len(unit_info)>0:
        flag = True

    return flag, unit_info

# 根据作战单元ID列表获取其完整实体信息列表
def GetUnitInfoArrayByIDList(df, unit_id_list):
    flag = True
    unit_info_list = []
    unit_info_dataframe = None

    for unit_id in unit_id_list:
        info_flag, unit_info = GetUnitInfoByID(df, unit_id)
        if info_flag:
            unit_info_list.append(unit_info)
        else:
            flag = False
            print('unit id is invalid, please check!')
            break

    if flag:
        unit_info_dataframe = pd.concat(unit_info_list, axis=0, ignore_index=True)

    return flag, unit_info_dataframe

# 判断作战单元是否被锁定
def UnitIsLockedByID(df, unit_id):
    flag = False
    unit_is_locked = df[df.ID == unit_id]['Locked'].values.tolist()

    if len(unit_is_locked)>0:
        flag = True

    return flag, unit_is_locked

# 根据作战单元ID列表判断其锁定状态列表
def GetUnitsLockedStateByIDList(df, unit_id_list):
    flag = False
    unit_locked_state_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_locked_state = UnitIsLockedByID(df, unit_id)
        if pos_flag:
            unit_locked_state_list.append(unit_locked_state[0])
            pos_flag = True
        else:
            unit_locked_state_list.append(None)
            pos_flag = False

    return flag, unit_locked_state_list

# 根据作战单元ID获取指定范围的特定类型作战单元
def GetUnitSetsByRange(df1, df2, unit_id, unit_type, radius, dim):
    # unit_id是df1里的记录
    # df2可以指定是己方作战单元信息还是qb态势信息
    unit_id_list = []

    target_id_list = df2[df2.LX == unit_type]['ID'].values.tolist()

    for target_id in target_id_list:
        _, dist = CalcDistByTwoUnitID(df1, df2, unit_id, target_id, dim)

        if dist <= radius:
            unit_id_list.append(target_id)

    return unit_id_list

# 根据方向和位置计算新的坐标点(二维)
def CalcDesPtByAzRadius(orgin_pt, az, radius):
    # az 角度范围 [-180.0,180.0]
    # 角度转弧度
    az_rad = az*math.pi/180.0
    return [orgin_pt[0]+radius*math.sin(az_rad), orgin_pt[1]+radius*math.cos(az_rad)]

# 根据作战单元ID推算其指定距离和方位的目标点
def CalcDesPtbyUnitID(df, unit_id, az, radius):
    pos_flag, unit_pos = GetUnitPosByID(df, unit_id, 2)

    if pos_flag:
        return CalcDesPtByAzRadius(unit_pos[0], az, radius)
    else:
        return None

# 根据两个作战单元的相对方位推算指定距离和方位的目标点
def CalcDesPtByTwoUnitID(df1, df2, unit1_id, unit2_id, radius):
    flag, az = CalcAzByTwoUnitID(df1, df2, unit1_id, unit2_id, 2)

    if flag:
        return CalcDesPtbyUnitID(df1, unit1_id, az, radius)
    else:
        return None

# 根据两组作战单元集合的相对方位推算指定距离和方位的目标点
def CalcDesPtArrayByTwoUnitIDList(df1, df2, unit_id_list1, unit_id_list2, radius):
    flag = True
    des_pt_list = []
    for unit1_id in unit_id_list1:
        for unit2_id in unit_id_list2:
            des_pt = CalcDesPtByTwoUnitID(df1, df2, unit1_id, unit2_id, radius)

            if des_pt is None:
                des_pt_list.append([None, None])
                flag = False
            else:
                des_pt_list.append(des_pt)


    return flag, np.array(des_pt_list).reshape((len(unit_id_list1), len(unit_id_list2)))


# 计算两个作战单元的距离
def CalcDistByTwoUnitID(df1, df2, unit1_id, unit2_id, dim):
    # dim为欧式距离的维数
    flag = False
    dist = None
    pos1_flag, unit1_pos = GetUnitPosByID(df1, unit1_id, dim)
    pos2_flag, unit2_pos = GetUnitPosByID(df2, unit2_id, dim)

    if pos1_flag == True and pos2_flag == True:
        if dim==2:
            dist = calculate_2d_distance(unit1_pos[0], unit2_pos[0])
        else:
            dist = calculate_3d_distance(unit1_pos[0], unit2_pos[0])
        flag = True

    return flag, dist

# 计算两组作战单元列表的相对距离
def CalcRelDistByTwoUnitIDList(df1, df2, id_list1, id_list2, dim):
    # dim为欧式距离的维数
    # 返回类型ndarray
    flag = False
    rel_dist_array = None

    pos1_flag, pos_list1 = GetUnitsPosByIDList(df1, id_list1, dim)
    pos2_flag, pos_list2 = GetUnitsPosByIDList(df2, id_list2, dim)

    if pos1_flag == True and pos2_flag == True:
        dist_array = cdist(np.array(pos_list1), np.array(pos_list2), metric='euclidean')
        flag = True
        dist_list = list(chain(*dist_array))
        a2a_idtuple = [(id1, id2) for id1 in id_list1 for id2 in id_list2]
        rel_dist_array = [[dist_list[i], a2a_idtuple[i]] for i in range(len(a2a_idtuple))]
        #print("rel_dist_array=",rel_dist_array)

    return flag, rel_dist_array

# 计算两个点的相对方向
def CalcAzByTwoPts(orgin_pt, des_pt):
    az_rad = math.atan2(des_pt[0]-orgin_pt[0], des_pt[1]-orgin_pt[1])

    return az_rad * 180.0 / math.pi

def CalcEdPosByDisAngle(dis,angle):
    az = angle* math.pi / 180.0
    x = dis * math.cos(az)
    y = dis * math.sin(az)
    return x,y

# 计算两个作战单元的相对方向 unit2在unit1的什么方位(以正北逆时针旋转)
def CalcAzByTwoUnitID(df1, df2, unit1_id, unit2_id, dim):
    # dim为欧式距离的维数 目前支持二维
    flag = False
    az = None
    pos1_flag, unit1_pos = GetUnitPosByID(df1, unit1_id, dim)
    pos2_flag, unit2_pos = GetUnitPosByID(df2, unit2_id, dim)

    if pos1_flag == True and pos2_flag == True:
        az = CalcAzByTwoPts(unit1_pos[0], unit2_pos[0])
        flag = True

    return flag, az

# 计算两个作战单元ID列表两两之间的相对方位
def CalcAzArrayByTwoUnitIDList(df1, df2, unit_id_list1, unit_id_list2):
    flag = True
    az_list = []
    for unit1_id in unit_id_list1:
        for unit2_id in unit_id_list2:
            az_flag, az = CalcAzByTwoUnitID(df1, df2, unit1_id, unit2_id, 2)
            if az_flag:
                az_list.append(az)
            else:
                az_list.append(None)
                flag = False

    return flag, np.array(az_list).reshape((len(unit_id_list1), len(unit_id_list2)))

# 根据作战单元的航向计算指定偏角对应的点
# def CalcDesPtByUnitHeading(df, unit_id, az, radius):
#     pos_flag, unit_pos = GetUnitPosByID(df, unit_id)
#
#     trans = np.array([])

# 根据作战单元的航向推算偏离航向一定角度指定距离的位置点
def CalcDesPtByUnitHeading(df, unit_id, delta_az, range):
    flag = False
    des_pt = None

    heading_flag, unit_heading = GetUnitHeadingByID(df, unit_id)
    pos_flag, unit_pos = GetUnitPosByID(df, unit_id, 2)

    if heading_flag:
        des_az = (unit_heading[0]+delta_az)*math.pi/180.0

        des_x, des_y = polarToCartesian(range, des_az)

        des_pt = [des_x+unit_pos[0][0], des_y+unit_pos[0][1]]

        flag = True
        return flag, des_pt

    else:
        return flag, des_pt

# 计算导弹预计到达目标的时间
def CalcMun2TargetArrivalTime(df, target_id, current_pt, speed, dim):
    flag = False
    arrival_time = None
    pos_flag, target_pos = GetUnitPosByID(df, target_id, dim)

    if pos_flag:
        if dim==2:
            dist = calculate_2d_distance(target_pos[0], current_pt)
        else:
            dist = calculate_3d_distance(target_pos[0], current_pt)

        arrival_time = dist/speed
        flag = True

        return flag, arrival_time

    else:
        return flag, arrival_time

def CalcP2PDistance(p1,p2,dim):
    flag = False
    if dim==2:
        dist = calculate_2d_distance(p1, p2)
        flag = True
    else:
        dist = calculate_3d_distance(p1, p2)
        flag = True
    return flag, dist

def point_segment_distance(px,py,sx1,sy1,sx2,sy2):
    dx = sx2-sx1
    dy = sy2-sy1
    som = dx**2 + dy**2
    u = 0
    if som > 0:
        u = ((px - sx1)*dx + (py - sy1)*dy) / som
    u = min(max(u,0),1)
    closestx = sx1 +u * dx
    closesty = sy1 +u * dy
    return ((px -closestx)**2 + (py -closesty)**2)**0.5

def test_point_segment_distance():
    #point (0,0) distance to segment (0,1)->(1,0)
    print(point_segment_distance(0,0,0,1,1,0)," = 0.707")
    # point (1,1) distance to segment (0,1)->(1,0)
    print(point_segment_distance(1,1,0,1,1,0)," = 0.707")

    # point (1,0) distance to segment (0,1)->(1,0)
    print(point_segment_distance(1,0,0,1,1,0)," = 0.0")
    # point (0,1) distance to segment (0,1)->(1,0)
    print(point_segment_distance(0,1,0,1,1,0)," = 0.0")
    # point (0.5,0.5) distance to segment (0,1)->(1,0)
    print(point_segment_distance(0.5,0.5,0,1,1,0)," = 0.0")

    # point (-1,2) distance to segment (0,1)->(1,0)
    print(point_segment_distance(-1,2,0,1,1,0)," = 1.414")
    # point (-1,2) distance to segment (0,1)->(1,0)
    print(point_segment_distance(2, -1, 0, 1, 1, 0), " = 1.414")

    # point (0,0) distance to segment (0,1)->(1,0)
    print(point_segment_distance(0, 0, 0, 1.732, 1, 0), " = ", 3**0.5/2)
    print(point_segment_distance(0, 0, 1, 0, 0, 1.732), " = ", 3 ** 0.5 / 2)


def point_segments_distance(px,py,sxs,sys):
    if len(sxs) != len(sys):
        return -1
    if len(sxs) == 0:
        return -1
    startx, starty = sxs[0], sys[0]
    distance = math.inf
    for i in range(len(sxs)):
        distance = min(distance, point_segment_distance(px, py, startx, starty, sxs[i], sys[i]))
        startx, starty = sxs[i], sys[i]

    return distance

def test_point_segments_distance():
    #point (0,0) distance to segment (0,1)->(1,0)->(2,0)
    print(point_segments_distance(0,0,[0,1,2],[1,0,0]), " = 0.707")
    #point (1,0) distance to segment (0,1)->(1,0)->(2,0)
    print(point_segments_distance(0,0,[0,1,2],[1,0,0]), " = 0.707")

    #point (0,0) distance to segment (0,1)->(1,0)->(0,2)
    print(point_segments_distance(0,0,[0,1,0],[1,0,2]), " = 0.707")
    #point (1,0) distance to segment (0,1)->(1,0)->(0,1)
    print(point_segments_distance(0,0,[0,1,0],[1,0,1]), " = 0.707")
    #point (1,0) distance to segment (0,1)->(1,0)->(0,1/2)
    print(point_segments_distance(0,0,[0,1,0],[1,0,1/2]), " = ",1/(5**0.5))


def CommandIsAliveByID(df, command_id):
    flag = False
    command_is_alive = None
    unit_alive_state = df[df.ID == command_id]['WH'].values.tolist()

    if len(unit_alive_state)>0:
        flag = True
        if unit_alive_state[0]==0:
            command_is_alive = False
        else:
            command_is_alive = True

        return flag, command_is_alive
    else:
        return flag, command_is_alive

# 获取指挥所存活数量
def GetCommandAliveNum(df):
    unit_alive_id_list = df[(df.LX == UnitType.COMMAND) & (df.WH == 1)]['ID'].tolist()

    return len(unit_alive_id_list)

# 将战损数据保存csv文件
def SaveCSVFile(log_name, jb_id, unique_token, data):
    path = Path('./results/' + log_name + '/' + unique_token)
    if not os.path.exists(path):
        os.makedirs(path)
    save = pd.DataFrame(data, columns = ['A2A', 'A2G', 'AWACS','DISTURB', 'UAV', 'SHIP','S2A', 'RADAR', 'COMMAND'])
    if jb_id == 0:
        save.to_csv(os.path.join(path, '%s_stat_result.csv'%'blue'), index = False, header = True)
    else:
        save.to_csv(os.path.join(path, '%s_stat_result.csv' % 'red'), index=False, header=True)

# 获取机场飞机数量
def GetAirportPlaneNum(airport_df):
    a2a_num = airport_df['AIR'].tolist()[0]
    awacs_num = airport_df['AWCS'].tolist()[0]
    jam_num = airport_df['JAM'].tolist()[0]
    uav_num = airport_df['UAV'].tolist()[0]
    a2g_num = airport_df['BOM'].tolist()[0]

    plane_dict = {'A2G':a2g_num, 'A2A': a2a_num, 'AWACS':awacs_num, 'DISTURB':jam_num, 'UAV':uav_num}

    return plane_dict

# 统计战损情况
def StatDamageUnits(unit_df, airport_df, jb_id, log_name, unique_token):
    # jb_id 0表示蓝方 1表示红方
    plane_dict = GetAirportPlaneNum(airport_df)

    # 统计歼击机损失数量
    stat_data = []
    _, a2a_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.A2A)
    if jb_id == 0:
        a2a_damage_num = BlueEntityNum.A2A - len(a2a_units_id_list) - plane_dict['A2A']
    else:
        a2a_damage_num = RedEntityNum.A2A - len(a2a_units_id_list) - plane_dict['A2A']

    stat_data.append(a2a_damage_num)

    # 统计轰炸机损失数量
    _, a2g_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.A2G)
    if jb_id == 0:
        a2g_damage_num = BlueEntityNum.A2G - len(a2g_units_id_list) - plane_dict['A2G']
    else:
        a2g_damage_num = RedEntityNum.A2G - len(a2g_units_id_list) - plane_dict['A2G']

    stat_data.append(a2g_damage_num)

    # 统计预警机损失数量
    _, awacs_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.AWACS)
    if jb_id == 0:
        awacs_damage_num = BlueEntityNum.AWACS - len(awacs_units_id_list) - plane_dict['AWACS']
    else:
        awacs_damage_num = RedEntityNum.AWACS - len(awacs_units_id_list) - plane_dict['AWACS']

    stat_data.append(awacs_damage_num)

    # 统计电子战飞机损失数量
    _, jammer_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.DISTURB)
    if jb_id == 0:
        jammer_damage_num = BlueEntityNum.DISTURB - len(jammer_units_id_list) - plane_dict['DISTURB']
    else:
        jammer_damage_num = RedEntityNum.DISTURB - len(jammer_units_id_list) - plane_dict['DISTURB']

    stat_data.append(jammer_damage_num)

    # 统计无人机损失数量
    _, uav_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.UAV)
    if jb_id == 0:
        uav_damage_num = BlueEntityNum.UAV - len(uav_units_id_list) - plane_dict['UAV']
    else:
        uav_damage_num = RedEntityNum.UAV - len(uav_units_id_list) - plane_dict['UAV']

    stat_data.append(uav_damage_num)

    # 统计舰艇损失数量
    _, ship_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.SHIP)
    if jb_id == 0:
        ship_damage_num = BlueEntityNum.SHIP - len(ship_units_id_list)
    else:
        ship_damage_num = RedEntityNum.SHIP - len(ship_units_id_list)

    stat_data.append(ship_damage_num)

    # 统计地防损失数量
    _, s2a_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.S2A)
    if jb_id == 0:
        s2a_damage_num = BlueEntityNum.S2A - len(s2a_units_id_list)
    else:
        s2a_damage_num = RedEntityNum.S2A - len(s2a_units_id_list)

    stat_data.append(s2a_damage_num)

    # 统计地面雷达损失数量
    _, radar_units_id_list = GetOwnUnitsID_ByType(unit_df, UnitType.RADAR)
    if jb_id == 0:
        radar_damage_num = BlueEntityNum.RADAR - len(radar_units_id_list)
    else:
        radar_damage_num = RedEntityNum.RADAR - len(radar_units_id_list)

    stat_data.append(radar_damage_num)

    # 统计指挥所数量
    if jb_id == 0:
        command_damage_num = BlueEntityNum.COMMAND - GetCommandAliveNum(unit_df)
    else:
        command_damage_num = 0

    stat_data.append(command_damage_num)

    SaveCSVFile(log_name, jb_id, unique_token, np.array(stat_data).reshape(-1, len(stat_data)))

    return

# add by lq
def save_mun_info(df, path):
    alg_run_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    unique_token = "{}".format(alg_run_datetime)
    path = path + unique_token

    if not os.path.exists(path):
        os.makedirs(path)

    df.to_csv(os.path.join(path, 'mun.csv'), index=False, header=True)

def calc_non_escape_zone(mun_speed, df, target_id, air_defense_range):
    # get target speed
    _, target_type = GetUnitTypeByID(df, target_id)


    if target_type[0] == UnitType.A2A:
        target_speed = TargetMaxSpeed.A2A
        k = AirDefenseCoefficient.A2A_COEFF

    if target_type[0] == UnitType.A2G:
        target_speed = TargetMaxSpeed.A2G
        k = AirDefenseCoefficient.A2G_COEFF

    if target_type[0] == UnitType.AWACS:
        target_speed = TargetMaxSpeed.AWACS
        k = AirDefenseCoefficient.AWACS_COEFF

    if target_type[0] == UnitType.DISTURB:
        target_speed = TargetMaxSpeed.DISTURB
        k = AirDefenseCoefficient.DISTURB_COEFF

    # Modified by lq 20200910
    if target_type[0] == UnitType.UNAIRTAR:
        target_speed = TargetMaxSpeed.UNAIRTAR
        k = AirDefenseCoefficient.UNAIRTAR_COEFF

    return (k * mun_speed * air_defense_range) / (mun_speed + target_speed)

def target_is_in_non_escape_zone(df1, air_defense_id, df2, target_id):
    # get air_defense type
    flag, air_defense_type = GetUnitTypeByID(df1, air_defense_id)

    if len(air_defense_type) == 0:
        print(df1)
        print(flag)
        print(air_defense_id)

    if air_defense_type[0] == UnitType.SHIP:
        air_defense_range = AirDefenseMunRange.SHIP_BOARD
        mun_speed = AirDefenseMunSpeed.SHIP_BOARD
    else:
        air_defense_range = AirDefenseMunRange.SHORE_BOARD
        mun_speed = AirDefenseMunSpeed.SHORE_BOARD

    # calc distance between air defense and target
    _, dist = CalcDistByTwoUnitID(df1, df2, air_defense_id, target_id, 2)

    non_escape_dist = calc_non_escape_zone(mun_speed, df2, target_id, air_defense_range)

    ratio =  dist / non_escape_dist

    return ratio, dist

def air_defense_filter_targets(df1, air_defense_id_list, df2, target_id_list, target_assign_history_df, sim_time):
    # exclude assigned target
    filter_target_assign_id_list = target_assign_history_df['target_id'].tolist()

    # if len(target_assign_history_df)>0:
    #     print('aa')
    air_defense_target_pairs = pd.DataFrame([], columns=['air_defense_type', 'air_defense_id', 'target_type', 'target_id', 'ratio', 'sim_time', 'dist'])
    target_sets = set()
    for air_defense_id in air_defense_id_list:
        _, air_defense_type = GetUnitTypeByID(df1, air_defense_id)
        for target_id in target_id_list:
            ratio, dist = target_is_in_non_escape_zone(df1, air_defense_id, df2, target_id)

            if ratio < 1 and target_id not in filter_target_assign_id_list:
                print('add new target_id:%d'%target_id)
                print(filter_target_assign_id_list)
                target_sets.add(target_id)
                _, target_type = GetUnitTypeByID(df2, target_id)
                air_defense_target_pairs = air_defense_target_pairs.append(
                    [{'air_defense_type': air_defense_type[0], 'air_defense_id': air_defense_id,
                      'target_type': target_type[0], 'target_id': target_id,
                      'ratio': ratio, 'sim_time':sim_time, 'dist':dist}], ignore_index=True)

    # if len(air_defense_target_pairs) != len(target_sets):
    #     assert False

    # sort
    # air_defense_target_pairs = air_defense_target_pairs.sort_values(axis=0, ascending=True, by=['ratio'])
    air_defense_target_pairs = air_defense_target_pairs.sample(frac=1)
    return air_defense_target_pairs

def air_defense_assignment(air_defense_df, air_defense_target_pairs):
    new_target_assign = pd.DataFrame([], columns = ['target_id', 'target_type', 'air_defense_id', 'air_defense_type', 'start_time', 'end_time'])
    # get idle channels of every air defense
    idle_channels = air_defense_df['idle_channel_num']
    air_defense_ids = air_defense_df['air_defense_id']
    # ['target_id', 'target_type', 'air_defense_id', 'air_defense_type', 'start_time', 'end_time']
    for index, row  in air_defense_target_pairs.iterrows():
        air_defense_id = row['air_defense_id']
        air_defense_type = row['air_defense_type']
        target_id = row['target_id']
        target_type = row['target_type']
        sim_time = row['sim_time']
        dist = row['dist']

        # air_defense_id = pair['ratio'].tolist()[0]
        # channel_index = np.argwhere(air_defense_ids == air_defense_id)[0][0]
        # print(idle_channels)
        # print('index:%d', channel_index)
        # print('idle_num:%d, target_id:%d' % (idle_channels[channel_index], target_id))
        # ad_df.loc[ad_df['air_defense_id'] == air_defense_id, ['idle_channel_num']]
        channel_num = air_defense_df.loc[air_defense_df['air_defense_id'] == air_defense_id, ['idle_channel_num']]
        print('idle_channel:')
        print(channel_num)
        if channel_num['idle_channel_num'].tolist()[0] > 0:
            air_defense_df.loc[air_defense_df['air_defense_id'] == air_defense_id, ['idle_channel_num']] = channel_num['idle_channel_num'].tolist()[0] - 1

            # calculate arrival time of mun
            if air_defense_type == UnitType.S2A:
                speed = AirDefenseMunSpeed.SHORE_BOARD
            if air_defense_type == UnitType.SHIP:
                speed = AirDefenseMunSpeed.SHIP_BOARD

            finish_time = sim_time + dist / speed

            # assign target
            new_target_assign = new_target_assign.append([{'target_type': target_type, 'target_id': target_id,
                                                                          'air_defense_type': air_defense_type,'air_defense_id':air_defense_id,
                                                                          'start_time':sim_time, 'end_time': finish_time}], ignore_index=True)



    return new_target_assign, air_defense_df

def GetUnitTypeByID(df, unit_id):
    flag = False

    unit_type = df[df.ID == unit_id]['LX'].values.tolist()

    if len(unit_type)>0:
        flag = True

    return flag, unit_type

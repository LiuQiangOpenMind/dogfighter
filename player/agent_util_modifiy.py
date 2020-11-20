from .agent_util import *
import random


class UnitAttrInfo:
    def __init__(self, Pos_X, Pos_Y, Pos_Z, Type, Speed, Heading, State, MunNum, TeamID):
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Pos_Z = Pos_Z
        self.Type = Type
        self.Speed = Speed
        self.Heading = Heading
        self.State = State
        self.MunNum = MunNum
        self.TeamID = TeamID

class EnemyAttrInfo:
    def __init__(self, Pos_X, Pos_Y, Pos_Z, Type, Speed, Heading, Side, Alive):
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Pos_Z = Pos_Z
        self.Type = Type
        self.Speed = Speed
        self.Heading = Heading
        self.Side = Side
        self.isAlive = Alive

class MunAttrInfo:
    def __init__(self, Pos_X, Pos_Y, Pos_Z, Heading):
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Pos_Z = Pos_Z
        self.Heading = Heading

class AirDefenseInfo:
    def __init__(self, self_type, idle_channel_num, used_channel_num):
        self.Type = self_type
        self.IdleChannelNum = idle_channel_num
        self.UsedChannelNum = used_channel_num
        self.LastOptStep = 0
        self.OnOff = True

class AirDefenseAssignInfo:
    def __init__(self, air_defense_id, start_time, end_time):
        self.air_defense_id = air_defense_id
        self.start_time = start_time
        self.end_time = end_time

class AssignPairInfo:
    def __init__(self, air_defense_id, target_id, dist):
        self.air_defense_id = air_defense_id
        self.target_id = target_id
        self.dist = dist

# 编队类型
def Dict_GetOwnUnitsData(units_list_dict):
    AllUnitsData = dict()

    for unit_dict in units_list_dict:
        if unit_dict['LX'] == UnitType.COMMAND:
            AllUnitsData[unit_dict['ID']] = UnitAttrInfo(unit_dict['X'], unit_dict['Y'],
                                                         unit_dict['Z'], unit_dict['LX'],
                                                         0.0, 0.0, 0, 0, -1)
        else:
            if len(list(unit_dict['WP'].values())) == 0:
                mun_num = -1
            else:
                mun_num = list(unit_dict['WP'].values())[0]
            AllUnitsData[unit_dict['ID']] = UnitAttrInfo(unit_dict['X'], unit_dict['Y'],
                                                         unit_dict['Z'], unit_dict['LX'],
                                                         unit_dict['SP'], unit_dict['HX'],
                                                         unit_dict['ST'], mun_num, unit_dict['TMID'])

    return AllUnitsData

def Dict_GetUnitInfo(data_dict, ID):
    if ID in data_dict:
        return True, data_dict[ID]
    else:
        return False, None

# 获取敌方数据
def Dict_GetQBData(qb_list_dict):
    AllQBData = dict()

    for qb_dict in qb_list_dict:
        # if qb_dict['LX'] == UnitType.COMMAND:
        #     continue

        AllQBData[qb_dict['ID']] = EnemyAttrInfo(qb_dict['X'], qb_dict['Y'], qb_dict['Z'], qb_dict['LX'],
                                                 qb_dict['SP'], qb_dict['HX'], qb_dict['JB'], qb_dict['WH'])

    return AllQBData
# 获取敌方航向数据
def Dict_GetEnemyMunData(mun_list_dict):
    AllMunsData = dict()

    for mun_dict in mun_list_dict:
        AllMunsData[mun_dict['ID']] = MunAttrInfo(mun_dict['X'], mun_dict['Y'], mun_dict['Z'], mun_dict['HX'])

    return AllMunsData
# 根据id获取类型
def Dict_GetUnitTypeByID(data_dict, ID):
    if ID in data_dict:
        unit_info = data_dict[ID]
        return True, unit_info.Type
    else:
        return False, None
# 根据类型获取我方id列表
def Dict_GetOwnUnitsID_ByType(data_dict, type):
    flag = False
    id_list = []
    for (k,v) in data_dict.items():
        if v.Type == type:
            id_list.append(k)

    if len(id_list) > 0:
        flag = True

    return flag, id_list
# 根据类型获取敌方id列表
def Dict_GetQBUnitsID_ByType(qb_data_dict, type, side):
    flag = False
    id_list = []
    for (k,v) in qb_data_dict.items():
        if v.Type == type and v.Side == side:
            id_list.append(k)

    if len(id_list) > 0:
        flag = True

    return flag, id_list
# 根据id获取坐标
def Dict_GetUnitPosByID(data_dict, unit_id, dim):
    if unit_id in data_dict:
        unit_info = data_dict[unit_id]
        if dim == 2:
            return True, [unit_info.Pos_X, unit_info.Pos_Y]
        else:
            return True, [unit_info.Pos_X, unit_info.Pos_Y, unit_info.Pos_Z]
    else:
        return False, None

def Dict_GetUnitTeamIDByID(data_dict, unit_id):
    if unit_id in data_dict:
        unit_info = data_dict[unit_id]
        return True, unit_info.TeamID
    else:
        return False, None

# 根据id获取状态
def Dict_GetUnitStateByID(data_dict, unit_id):
    if unit_id in data_dict:
        unit_info = data_dict[unit_id]
        return True, unit_info.State
    else:
        return False, None

def Dict_GetUnitHeadingByID(data_dict, unit_id):
    if unit_id in data_dict:
        unit_info = data_dict[unit_id]
        return True, unit_info.Heading
    else:
        return False, None

def Dict_GetUnitSpeedByID(data_dict, unit_id):
    if unit_id in data_dict:
        unit_info = data_dict[unit_id]
        return True, unit_info.Speed
    else:
        return False, None

def Dict_GetUnitMunNumByID(data_dict, unit_id):
    if unit_id in data_dict:
        unit_info = data_dict[unit_id]
        return True, unit_info.MunNum
    else:
        return False, None

def Dict_GetUnitsSpeedByIDList(data_dict, unit_id_list):
    flag = True
    unit_speed_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_speed = Dict_GetUnitSpeedByID(data_dict, unit_id)
        unit_speed_list.append(unit_speed)

        if pos_flag == False:
            flag = False

    return flag, unit_speed_list

def Dict_GetUnitsMunNumByIDList(data_dict, unit_id_list):
    flag = True
    unit_mun_num_list = []

    for unit_id in unit_id_list:
        mun_num_flag, unit_mun_num = Dict_GetUnitMunNumByID(data_dict, unit_id)
        unit_mun_num_list.append(unit_mun_num)

        if mun_num_flag == False:
            flag = False

    return flag, unit_mun_num_list

def Dict_GetUnitsPosByIDList(data_dict, unit_id_list, dim):
    flag = True
    unit_pos_list = []

    for unit_id in unit_id_list:
        pos_flag, unit_pos = Dict_GetUnitPosByID(data_dict, unit_id, dim)

        if pos_flag:
            unit_pos_list.append(unit_pos)
        else:
            if dim == 2:
                unit_pos_list.append([None ,None])
            else:
                unit_pos_list.append([None, None, None])

            flag = False

    return flag, unit_pos_list


def Dict_CalcRelDistByTwoUnitIDList(data_dict1, data_dict2, id_list1, id_list2, dim):
    # dim为欧式距离的维数
    # 返回类型ndarray
    flag = False
    rel_dist_array = None

    pos1_flag, pos_list1 = Dict_GetUnitsPosByIDList(data_dict1, id_list1, dim)
    pos2_flag, pos_list2 = Dict_GetUnitsPosByIDList(data_dict2, id_list2, dim)

    if pos1_flag == True and pos2_flag == True:
        dist_array = cdist(np.array(pos_list1), np.array(pos_list2), metric='euclidean')
        flag = True
        dist_list = list(chain(*dist_array))
        a2a_idtuple = [(id1, id2) for id1 in id_list1 for id2 in id_list2]
        rel_dist_array = [[dist_list[i], a2a_idtuple[i]] for i in range(len(a2a_idtuple))]
        #print("rel_dist_array=",rel_dist_array)

    return flag, rel_dist_array

def Dict_CalcDistByTwoUnitID(data_dict1, data_dict2, unit1_id, unit2_id, dim):
    # dim为欧式距离的维数
    flag = False
    dist = None
    pos1_flag, unit1_pos = Dict_GetUnitPosByID(data_dict1, unit1_id, dim)
    pos2_flag, unit2_pos = Dict_GetUnitPosByID(data_dict2, unit2_id, dim)

    if pos1_flag == True and pos2_flag == True:
        if dim==2:
            dist = calculate_2d_distance(unit1_pos, unit2_pos)
        else:
            dist = calculate_3d_distance(unit1_pos, unit2_pos)
        flag = True

    return flag, dist

def Dict_CalcRelDistByTwoUnitIDList(data_dict1, data_dict2, id_list1, id_list2, dim):
    # dim为欧式距离的维数
    # 返回类型ndarray
    flag = False
    rel_dist_array = None

    pos1_flag, pos_list1 = Dict_GetUnitsPosByIDList(data_dict1, id_list1, dim)
    pos2_flag, pos_list2 = Dict_GetUnitsPosByIDList(data_dict2, id_list2, dim)

    if pos1_flag == True and pos2_flag == True:
        dist_array = cdist(np.array(pos_list1), np.array(pos_list2), metric='euclidean')
        flag = True
        dist_list = list(chain(*dist_array))
        a2a_idtuple = [(id1, id2) for id1 in id_list1 for id2 in id_list2]
        rel_dist_array = [[dist_list[i], a2a_idtuple[i]] for i in range(len(a2a_idtuple))]
        #print("rel_dist_array=",rel_dist_array)

    return flag, rel_dist_array
# 计算两个点的相对方向
def Dict_CalcAzByTwoUnitID(data_dict1, data_dict2, unit1_id, unit2_id, dim):
    # dim为欧式距离的维数 目前支持二维
    flag = False
    az = None
    pos1_flag, unit1_pos = Dict_GetUnitPosByID(data_dict1, unit1_id, dim)
    pos2_flag, unit2_pos = Dict_GetUnitPosByID(data_dict2, unit2_id, dim)

    if pos1_flag == True and pos2_flag == True:
        az = CalcAzByTwoPts(unit1_pos, unit2_pos)
        flag = True

    return flag, az
#
def Dict_CalcAzArrayByTwoUnitIDList(data_dict1, data_dict2, unit_id_list1, unit_id_list2):
    flag = True
    az_list = []
    for unit1_id in unit_id_list1:
        for unit2_id in unit_id_list2:
            az_flag, az = Dict_CalcAzByTwoUnitID(data_dict1, data_dict2, unit1_id, unit2_id, 2)
            if az_flag:
                az_list.append(az)
            else:
                az_list.append(None)
                flag = False

    return flag, np.array(az_list).reshape((len(unit_id_list1), len(unit_id_list2)))

def Dict_CalcDesPtByUnitHeading(data_dict, unit_id, delta_az, range):
    flag = False
    des_pt = None

    heading_flag, unit_heading = Dict_GetUnitHeadingByID(data_dict, unit_id)
    pos_flag, unit_pos = Dict_GetUnitPosByID(data_dict, unit_id, 2)

    if heading_flag:
        des_az = (unit_heading+delta_az)*math.pi/180.0

        des_x, des_y = polarToCartesian(range, des_az)

        des_pt = [des_x+unit_pos[0], des_y+unit_pos[1]]

        flag = True
        return flag, des_pt

    else:
        return flag, des_pt

def dict_calc_non_escape_zone(mun_speed, target_data_dict, target_id, air_defense_range):
    # get target speed
    _, target_type = Dict_GetUnitTypeByID(target_data_dict, target_id)


    # if target_type[0] == UnitType.A2A:
    #     target_speed = TargetMaxSpeed.A2A
    #     k = AirDefenseCoefficient.A2A_COEFF
    # only A2G AWACS DISTURB
    if target_type[0] == UnitType.A2G:
        target_speed = TargetMaxSpeed.A2G
        k = AirDefenseCoefficient.A2G_COEFF

    if target_type[0] == UnitType.AWACS:
        target_speed = TargetMaxSpeed.AWACS
        k = AirDefenseCoefficient.AWACS_COEFF

    if target_type[0] == UnitType.DISTURB:
        target_speed = TargetMaxSpeed.DISTURB
        k = AirDefenseCoefficient.DISTURB_COEFF
    else:
        assert False
    # Modified by lq 20200910
    # if target_type[0] == UnitType.UNAIRTAR:
    #     target_speed = TargetMaxSpeed.UNAIRTAR
    #     k = AirDefenseCoefficient.UNAIRTAR_COEFF

    return (k * mun_speed * air_defense_range) / (mun_speed + target_speed)

# Modified ???
def dict_target_is_in_non_escape_zone(data_dict1, air_defense_id, data_dict2, target_id):
    # get air_defense type
    flag, air_defense_type = Dict_GetUnitTypeByID(data_dict1, air_defense_id)

    if len(air_defense_type) == 0:
        print(data_dict1)
        print(flag)
        print(air_defense_id)

    if air_defense_type == UnitType.SHIP:
        air_defense_range = AirDefenseMunRange.SHIP_BOARD
        mun_speed = AirDefenseMunSpeed.SHIP_BOARD
    else:
        air_defense_range = AirDefenseMunRange.SHORE_BOARD
        mun_speed = AirDefenseMunSpeed.SHORE_BOARD

    # calc distance between air defense and target
    _, dist = Dict_CalcDistByTwoUnitID(data_dict1, data_dict2, air_defense_id, target_id, 2)

    non_escape_dist = dict_calc_non_escape_zone(mun_speed, data_dict2, target_id, air_defense_range)

    ratio =  dist / non_escape_dist

    return ratio, dist

# Modified???
def dict_air_defense_filter_targets(df1, air_defense_id_list, df2, target_id_list, target_assign_history_df, sim_time):
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

# Modified???
def dict_air_defense_assignment(air_defense_df, air_defense_target_pairs):
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

def dict_calc_non_escape_zone(mun_speed, target_data_dict, target_id, air_defense_range):
    # get target speed
    _, target_type = Dict_GetUnitTypeByID(target_data_dict, target_id)

    if target_type == UnitType.A2A:
        target_speed = TargetMaxSpeed.A2A
        k = AirDefenseCoefficient.A2A_COEFF
    elif target_type == UnitType.A2G:
        target_speed = TargetMaxSpeed.A2G
        k = AirDefenseCoefficient.A2G_COEFF
    elif target_type == UnitType.AWACS:
        target_speed = TargetMaxSpeed.AWACS
        k = AirDefenseCoefficient.AWACS_COEFF

    elif target_type == UnitType.DISTURB:
        target_speed = TargetMaxSpeed.DISTURB
        k = AirDefenseCoefficient.DISTURB_COEFF
    else:
        print('type:')
        assert False


    return (k * mun_speed * air_defense_range) / (mun_speed + target_speed)

# Modified ???
def dict_target_is_in_non_escape_zone(data_dict1, air_defense_id, data_dict2, target_id):
    # get air_defense type
    flag, air_defense_type = Dict_GetUnitTypeByID(data_dict1, air_defense_id)

    if air_defense_type == UnitType.SHIP:
        air_defense_range = AirDefenseMunRange.SHIP_BOARD
        mun_speed = AirDefenseMunSpeed.SHIP_BOARD
    else:
        air_defense_range = AirDefenseMunRange.SHORE_BOARD
        mun_speed = AirDefenseMunSpeed.SHORE_BOARD

    # calc distance between air defense and target
    _, dist = Dict_CalcDistByTwoUnitID(data_dict1, data_dict2, air_defense_id, target_id, 2)

    non_escape_dist = dict_calc_non_escape_zone(mun_speed, data_dict2, target_id, air_defense_range)

    ratio =  dist / non_escape_dist

    return ratio, dist

# Modified???
def dict_air_defense_filter_targets(data_dict1, air_defense_id_list, data_dict2, target_id_list, target_assign_history_dict, sim_time):
    # exclude assigned target
    filter_target_assign_id_list = list(target_assign_history_dict.keys())

    print('current targets:')
    print(target_id_list)
    print('assigned targets:')
    print(filter_target_assign_id_list)
    air_defense_target_pairs = dict()
    target_sets = set()
    for air_defense_id in air_defense_id_list:
        _, air_defense_type = Dict_GetUnitTypeByID(data_dict1, air_defense_id)
        for target_id in target_id_list:
            _, target_type = Dict_GetUnitTypeByID(data_dict2, target_id)

            if target_type == UnitType.UNAIRTAR:
                continue

            ratio, dist = dict_target_is_in_non_escape_zone(data_dict1, air_defense_id, data_dict2, target_id)

            if ratio < 1 and target_id not in filter_target_assign_id_list:


                target_sets.add(target_id)

                air_defense_target_pairs[ratio] = AssignPairInfo(air_defense_id, target_id, dist)
    if len( air_defense_target_pairs) > 0:
        print('air_defense_target_pairs condition:')
    for (k,v) in  air_defense_target_pairs.items():
        print('ratio: %f air_defense_id: %d target_id: %d dist:%f' %(k, v.air_defense_id, v.target_id, v.dist))

    return air_defense_target_pairs

# Modified???
def dict_air_defense_assignment(air_defense_dict, air_defense_target_pairs, sim_time, shuffle):
    new_target_assign = dict()
    assigned_targets = []

    # 排序 dict to list
    air_defense_target_pairs = sorted(air_defense_target_pairs.items(), key=lambda x: x[0])
    print('air_defense_target_pairs sorted:')
    for assign_pair_tuple in air_defense_target_pairs:
        print('ratio: %f air_defense_id: %d target_id: %d dist:%f' % (assign_pair_tuple[0], assign_pair_tuple[1].air_defense_id, assign_pair_tuple[1].target_id, assign_pair_tuple[1].dist))
    # print(air_defense_target_pairs)
    if shuffle:
        random.shuffle(air_defense_target_pairs)

    print('air_defense_target_pairs shuffle:')
    for assign_pair_tuple in air_defense_target_pairs:
        print('ratio: %f air_defense_id: %d target_id: %d dist:%f' % (assign_pair_tuple[0], assign_pair_tuple[1].air_defense_id, assign_pair_tuple[1].target_id, assign_pair_tuple[1].dist))
    for assign_pair_tuple in air_defense_target_pairs:
        if assign_pair_tuple[1].target_id in assigned_targets:
            continue

        air_defense_info = air_defense_dict[assign_pair_tuple[1].air_defense_id]

        if air_defense_info.IdleChannelNum > 0:
            air_defense_info.IdleChannelNum -= 1

            if air_defense_info.Type == UnitType.S2A:
                speed = AirDefenseMunSpeed.SHORE_BOARD
            if air_defense_info.Type == UnitType.SHIP:
                speed = AirDefenseMunSpeed.SHIP_BOARD

            finish_time = sim_time + assign_pair_tuple[1].dist / speed
            assigned_targets.append(assign_pair_tuple[1].target_id)
            new_target_assign[assign_pair_tuple[1].target_id] = AirDefenseAssignInfo(assign_pair_tuple[1].air_defense_id, sim_time, finish_time)
    print('air_defense_target_pairs:')
    for (k, v) in new_target_assign.items():
        print('target_id: %d air_defense_id: %d sim_time: %f finish_time:%f' % (k, v.air_defense_id, v.start_time, v.end_time))
    return new_target_assign






























from enum import Enum
import math, json

from .agent import Agent
from env.env_cmd import EnvCmd
from env.env_def import UnitType, UnitStatus, RED_AIRPORT_ID
from .MinMaxWeigthMatching import *
from .agent_util import *
from .agent_util_modifiy import *
from .MinMaxCluster_util import maxmin_distance_cluster
import time
import random as rand
import math


DISTURB_WEIGHT = 15
A2A_WEIGHT = 1
A2G_WEIGHT = 2
AWACS_WEIGHT = 4
UNAIRTAR_WEIGHT = 2
SCALE_FACTOR = 2
DENFENSE_RANGE_A2A_1 = 30000
DENFENSE_RANGE_A2A_2 = 50000

SHIP_PATROL_PARAMS_0 = [270, 5000, 5000, 19.99, 3600, 0]

# 0910
SHIP_TO_CMD_DIS = 45000
SHIP_TO_CMD_ANGLE = -30 * math.pi / 180
Fighter_offset = 45000

NORTH_COMMAND_POS = [-129532, 87667, 0]
SOUTH_COMMAND_POS = [-131154, -87888, 0]
NEW_NORTH_COMMAND_POS = [-250830, 184288, 0]
NEW_SOUTH_COMMAND_POS = [-295519, -100815, 0]
NORTH_FIRST_S2A_POS = [-232595, 182906, 0]
NORTH_SECOND_S2A_POS = [-232595, 192906, 0]
SOUTH_FIRST_S2A_POS = [-285888, -79738, 0]
SOUTH_SECOND_S2A_POS = [-285888, -80738, 0]
AIRPORT_POS=[-266405.54, 7976.405, 18]
#test_point = [-8000,0]
SHIP_POINT = [NORTH_COMMAND_POS[0] + SHIP_TO_CMD_DIS * math.cos(SHIP_TO_CMD_ANGLE), NORTH_COMMAND_POS[1] + SHIP_TO_CMD_DIS * math.sin(SHIP_TO_CMD_ANGLE), 0]
FIRST_SHIP_POINT = [NEW_NORTH_COMMAND_POS[0] + SHIP_TO_CMD_DIS * math.cos(SHIP_TO_CMD_ANGLE), NEW_NORTH_COMMAND_POS[1] + SHIP_TO_CMD_DIS * math.sin(SHIP_TO_CMD_ANGLE), 0]
# FIRST_SHIP_POINT = NEW_NORTH_COMMAND_POS
SECOND_SHIP_POINT = [NEW_SOUTH_COMMAND_POS[0] + SHIP_TO_CMD_DIS * math.cos(-SHIP_TO_CMD_ANGLE), NEW_SOUTH_COMMAND_POS[1] + SHIP_TO_CMD_DIS * math.sin(-SHIP_TO_CMD_ANGLE), 0]
#SHIP_POINT = [test_point[0], test_point[1], 0]
THIRD_SHIP_POINT = [-350000, 0, 0]
#SHIP_POINT = [test_point[0], test_point[1], 0]

Fighter_MassArea_Params = [270, 5000, 5000, 1000 / 3.61, 7200]
FIGHTER_TO_SHIP_ANGLE = -30*math.pi/180
Fighter_Near_Ship_Pos = [FIRST_SHIP_POINT[0] + Fighter_offset * math.cos(FIGHTER_TO_SHIP_ANGLE * math.pi / 180), FIRST_SHIP_POINT[1] + Fighter_offset * math.sin(FIGHTER_TO_SHIP_ANGLE * math.pi / 180), 7500]
Fighter_Near_North_Ship_Pos = [FIRST_SHIP_POINT[0] + Fighter_offset * math.cos(FIGHTER_TO_SHIP_ANGLE), FIRST_SHIP_POINT[1] + Fighter_offset * math.sin(FIGHTER_TO_SHIP_ANGLE), 7500]
Fighter_Near_South_Ship_Pos = [SECOND_SHIP_POINT[0] + Fighter_offset * math.cos(-FIGHTER_TO_SHIP_ANGLE), SECOND_SHIP_POINT[1] + Fighter_offset * math.sin(-FIGHTER_TO_SHIP_ANGLE), 7500]
Fighter_Near_North_CMD_Pos = [NEW_NORTH_COMMAND_POS[0] + Fighter_offset * math.cos(FIGHTER_TO_SHIP_ANGLE), NEW_NORTH_COMMAND_POS[1] + Fighter_offset * math.sin(FIGHTER_TO_SHIP_ANGLE), 7500]
Fighter_Near_South_CMD_Pos = [NEW_SOUTH_COMMAND_POS[0] + Fighter_offset * math.cos(-FIGHTER_TO_SHIP_ANGLE), NEW_SOUTH_COMMAND_POS[1] + Fighter_offset * math.sin(-FIGHTER_TO_SHIP_ANGLE), 7500]
AWACS_PATROL_PARAMS = [270, 10000, 30000, 800 / 3.61, 7200, 2]

#9-14
Bomber_Mass_Point_Middle = [-40000, 0, 8000]     #轰炸机起飞巡逻位置
Bomber_MassArea_Params = [270, 20000, 20000, 800 / 3.61, 7200]
Bomber_Mass_Point = [-140000, 0, 8000]

# 9-16
ESCAPE_DISTANCE_RED_A2A = 105000
ESCAPE_DISTANCE_RED_SHIP = 150000
ESCAPE_range = 5000

AWACS_POS1 = [-90000, 0, 7500]
AWACS_POS2 = [-130000, 20000, 7500]
AWACS_POS3 = [-130000, -20000, 7500]

class Point():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class FighterCommand:
    def __init__(self, entity_id):
        self.entity_id = entity_id
        self.attack_fighter_cmd = None
        self.escape_cmd = None
        self.return_base_cmd = None
        self.patrol_cmd = None

class Missle:
    def __init__(self,targetId,deadTime):
        self.targetId = targetId
        self.deadTime = deadTime


class Missle_Manager:
    def __init__(self):
        self.items = []

    def addItem(self, targetId,deadTime):
        self.items.append(Missle(targetId,deadTime))

    def getAliveMissle(self, targetId, currentTime):
        result1 = 0
        for item in self.items:
            if item.targetId == targetId and currentTime < item.deadTime:
                result1 += 1
        return result1


class Side(Enum):
    RED = 1
    BLUE = 0
    UNKNOWN = 2

#9-14   bomber
class AttackAssignmentState(Enum):
    NO_ASSIGNMENT = 0
    ASSIGNMENT_WHEN_ONE_RED_SHIP = 1
    ASSIGNMENT_WHEN_TWO_RED_SHIP = 2

#9-14   bomber
class AttackAssignment:
    def __init__(self):
        self.attack_assignment_state = AttackAssignmentState.NO_ASSIGNMENT
        self.mount_num = 0

#0922  zh
class BomberCommand:
    def __init__(self, entity_id):
        self.entity_id = entity_id
        self.attack_ship_cmd = None
        self.escape_cmd = None
        self.return_base_cmd = None
        self.attack_path_cmd = None
        self.patrol_cmd = None

class NwgcBlueDefenseAgent(Agent):
    def __init__(self, name, config, **kwargs):
        super().__init__(name, config['side'])

        self._init()

    def _init(self):

        self.curr_time = 0
        self.agent_state = 0

        self.red_ship_ids = list()
        self.red_aircrafts_info_history = {}  # {id:type}
        self.green_aircrafts_info_history = {}  # {id:type}
        self.qb_aircrafts_info_current = {} # {type:set(id)}
        self.red_air_ids_current = set() #{id}
        self.red_air_ids_current_cluster_info = []

#20100910
        self.A2A_Missle_Manager = Missle_Manager()
        self.last_air_cmd_info = dict()#{id:(blue_fighter_id,red_air_id,fly_time,mount_num_when_cmd}

        # add by lq
        self.air_defense_assign_info_history = pd.DataFrame([], columns = ['target_id', 'target_type', 'air_defense_id', 'air_defense_type', 'start_time', 'end_time'])
        self.own_air_defense_df = pd.DataFrame([], columns=['air_defense_id', 'air_defense_type', 'idle_channel_num', 'current_mun_num'])

        # 20200914
        self.bomber_attack_assignment_dict = {}  # {id:AttackAssignment}

        # add by lq 20200915
        self.own_air_defense_dict = dict()
        self.air_defense_assign_history_dict = dict()


        # add by zh 20200922
        self.red_ship_info_cache_dict = dict()  #{id:[x,y,simTime]}
        self.usedIdx = []
        self.count_ship_set = 0
        self.last_bombers_attack = {}
        self.is_first_bomber_lunch = False

        # add by lq 20200923
        self.step_count = 0

        
    def reset(self):
        self._init()


    def step(self, sim_time, obs, **kwargs):
        import time
        start = time.time()
        # 更新units_dataframe
        self.dict_units = Dict_GetOwnUnitsData(obs['units'])
        # df_teams = GetOwnUnitsData(obs['teams'])
        #self.df_qb = GetQBData(obs['qb'])
        self.dict_qb = Dict_GetQBData(obs['qb'])
        # df_rockets = GetQBData(obs['rockets'])

        self.update_red_ships_detect_info()
        self.update_red_aircraft_detect_info()

        self.curr_time = sim_time  # 当前时间
        self.obs = obs
        cmd_list = []

        #takce off
        if self.agent_state == 0:
            index = 1
            _, s2a_id_list = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.S2A)
            _, s2a_pos_list = Dict_GetUnitsPosByIDList(self.dict_units, s2a_id_list, 3)
            for idx, id in enumerate(s2a_id_list):
                cmd_list.extend([EnvCmd.make_ground_movedeploy(id, s2a_pos_list[idx][0], s2a_pos_list[idx][1],
                                                               s2a_pos_list[idx][2], 90, 1)])

            # a2g 10 takeoff
            '''cmd_list.extend(
                self._takeoff_areapatrol(4, UnitType.A2G, Bomber_Mass_Point_Middle, Bomber_MassArea_Params))
            cmd_list.extend(
                self._takeoff_areapatrol(4, UnitType.A2G, Bomber_Mass_Point_Middle, Bomber_MassArea_Params))
            cmd_list.extend(
                self._takeoff_areapatrol(2, UnitType.A2G, Bomber_Mass_Point_Middle, Bomber_MassArea_Params))'''

            # a2a 20 takeoff
            cmd_list.extend(
                self._takeoff_areapatrol(4, UnitType.A2A, Fighter_Near_Ship_Pos, Fighter_MassArea_Params))
            cmd_list.extend(
                self._takeoff_areapatrol(4, UnitType.A2A, Fighter_Near_North_CMD_Pos, Fighter_MassArea_Params))
            cmd_list.extend(
                self._takeoff_areapatrol(4, UnitType.A2A, Fighter_Near_South_CMD_Pos, Fighter_MassArea_Params))
            cmd_list.extend(
                self._takeoff_areapatrol(4, UnitType.A2A, Fighter_Near_North_CMD_Pos, Fighter_MassArea_Params))
            cmd_list.extend(
                self._takeoff_areapatrol(4, UnitType.A2A, Fighter_Near_South_CMD_Pos, Fighter_MassArea_Params))

            # awacs patrol
            awacs_flag, awacs_ID = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.AWACS)
            if awacs_flag and self.curr_time <= 1200:
                cmd_list.extend(self._awacs_patrol(awacs_ID[0], AWACS_POS1, AWACS_PATROL_PARAMS))

            # ship location assignment
            cmd_list.extend(self.air_defense_deploy())

            self.air_defense_init()
            self.agent_state = 1
        # airport planning
        fighters_num_air = obs['airports'][0]['AIR']

        if fighters_num_air > 0 and sim_time > 600:
            cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2A, Fighter_Near_South_CMD_Pos,
                                                     Fighter_MassArea_Params))

        # if bomber_num_air > 0:
        #     cmd_list.extend(self._takeoff_areapatrol(bomber_num_air, UnitType.A2G, Bomber_Mass_Point_Middle,
        #                                              Bomber_MassArea_Params))

        cmd_list.extend(self.awacs_planning())
        #if self.curr_time > 10:
        #    cmd_list.extend(self.bombers_planning_ns())
        cmd_list.extend(self.ship_planning())
        self.global_planning()
        cmd_list.extend(self.fighters_planning())
        self.air_defense_update()
        cmd_list.extend(self.air_defense_planning4(sim_time))




        #wt测试
        #cmd_list.extend(self.ship_planning(sim_time))
        print("仿真运行时刻：",self.curr_time)
        print(time.time()-start,'蓝方step总体运行时间+++++++++++++++++++++++++++++++++++++++++++++++++++++++++')

        self.step_count += 1

        return cmd_list

    # def ship_planning(self,cur_time):
    #     manvcmdlist = []
    #     if cur_time>120:
    #         iflag, shipIDlist = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)
    #         if iflag:
    #             manvcmdlist.extend(self._ship_areapatrol(shipIDlist[0], [SHIP_POINT[0], SHIP_POINT[1], 0]))
    #     return manvcmdlist

    #
    def global_planning(self):
        #1 commands and ships state
        self.north_cmd_id = None
        self.south_cmd_id = None
        self.north_ship_id = None
        self.south_ship_id = None
        self.cmd_ship_count = 0
        currentPatrolPoss = []
        flag, ship_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)
        _flag, ship_Poss = Dict_GetUnitsPosByIDList(self.dict_units, ship_IDs, 2)

        flag, cmd_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.COMMAND)
        _flag,cmd_Poss = Dict_GetUnitsPosByIDList(self.dict_units, cmd_IDs,2)
        for cmd_idx,cmd_Pos in enumerate(cmd_Poss):
            self.cmd_ship_count = self.cmd_ship_count+1
            if calculate_2d_distance(cmd_Pos,NEW_NORTH_COMMAND_POS[0:2])<calculate_2d_distance(cmd_Pos,NEW_SOUTH_COMMAND_POS[0:2]):
                self.north_cmd_id=cmd_IDs[cmd_idx]
                self.north_cmd_pos = cmd_Pos
            else:
                self.south_cmd_id=cmd_IDs[cmd_idx]
                self.south_cmd_pos = cmd_Pos
        for ship_idx,ship_Pos in enumerate(ship_Poss):
            self.cmd_ship_count = self.cmd_ship_count + 1
            if calculate_2d_distance(ship_Pos,NEW_NORTH_COMMAND_POS[0:2])<calculate_2d_distance(ship_Pos,NEW_SOUTH_COMMAND_POS[0:2]):
                self.north_ship_id=ship_IDs[ship_idx]
                self.north_ship_pos = ship_Pos
            else:
                self.south_ship_id=ship_IDs[ship_idx]
                self.south_ship_pos = ship_Pos

        #2 air attack and denfense number assignment
        self.air_attack_percent = min(0,1/( self.cmd_ship_count+1))
        self.air_denfense_percent = 1-self.air_attack_percent

        flag, fighter_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.A2A)
        fighter_ids_with_mount = [fighter_idx for fighter_idx in fighter_IDs if
                                 Dict_GetUnitMunNumByID(self.dict_units, fighter_idx)[1] > 0]
        
        self.fighter_num = len(fighter_ids_with_mount)
        self.air_attack_num = int(self.air_attack_percent*self.fighter_num)
        self.air_denfense_num = self.fighter_num - self.air_attack_num

        #3 air attack poss planning

        #4 air denfense number assigment for cmds and ships
        self.north_cmd_enemy_threat_weight = 0.00000001 if self.north_cmd_id is  None else self.calculate_enemies_threat_weight(self.north_cmd_pos)
        self.south_cmd_enemy_threat_weight= 0.00000001 if self.south_cmd_id is None else self.calculate_enemies_threat_weight(self.south_cmd_pos)
        self.north_ship_enemy_threat_weight = 0.00000001 if self.north_cmd_id is None or self.north_ship_id is None else self.calculate_enemies_threat_weight(self.north_ship_pos)
        self.south_ship_enemy_threat_weight = 0.00000001 if self.south_cmd_id is None or self.south_ship_id is None else self.calculate_enemies_threat_weight(self.south_ship_pos)
        weights= [self.north_cmd_enemy_threat_weight,self.south_cmd_enemy_threat_weight,self.north_ship_enemy_threat_weight,self.south_ship_enemy_threat_weight]
        weight_sum = sum(weights)
        denfense_num_for_each_battle_field = (self.air_denfense_num*np.array(weights)/weight_sum).astype(int)
        #rest for most threat
        rest = self.air_denfense_num - sum(denfense_num_for_each_battle_field)
        denfense_num_for_each_battle_field[np.argmax(weights)] = denfense_num_for_each_battle_field[np.argmax(weights)]+rest
        print("A2A Assignments:",denfense_num_for_each_battle_field)

        # air defense poss planning
        defense_pos = []#[[x,y],...]
        THREAT_DIS=250000
        if self.north_cmd_id is not None:
            defense_pos.extend(self.calcualte_defense_position(self.north_cmd_pos,DENFENSE_RANGE_A2A_1,denfense_num_for_each_battle_field[0],THREAT_DIS))
        if self.south_cmd_id is not None:
            defense_pos.extend(self.calcualte_defense_position(self.south_cmd_pos,DENFENSE_RANGE_A2A_1,denfense_num_for_each_battle_field[1],THREAT_DIS))
        if self.north_ship_id is not None:
            defense_pos.extend(self.calcualte_defense_position(self.north_ship_pos,DENFENSE_RANGE_A2A_2,denfense_num_for_each_battle_field[2],THREAT_DIS))
        if self.south_ship_id is not None:
            defense_pos.extend(self.calcualte_defense_position(self.south_ship_pos,DENFENSE_RANGE_A2A_2,denfense_num_for_each_battle_field[3],THREAT_DIS))
        _,fighter_poss_with_mount = Dict_GetUnitsPosByIDList(self.dict_units, fighter_ids_with_mount, 2)
        self.fighter_id_pos_dict = {}
        if len(defense_pos)>0:
            result = minMaxWeightMatchingCppVer(fighter_poss_with_mount,defense_pos)
            print(result)
            print("***")
            for idx,id in enumerate(fighter_ids_with_mount):
                self.fighter_id_pos_dict[id]=defense_pos[result[idx][1]]


        #ship pos planning
        DENFENSE_RANGE_SHIP=35000
        self.north_ship_defense_pos = None
        self.south_ship_defense_pos = None
        if self.north_cmd_id is not None:
            self.north_ship_defense_pos =  self.calcualte_defense_position(self.north_cmd_pos, DENFENSE_RANGE_SHIP, 1, THREAT_DIS)[0]
        if self.south_cmd_id is not None:
            self.south_ship_defense_pos =  self.calcualte_defense_position(self.south_cmd_pos, DENFENSE_RANGE_SHIP, 1, THREAT_DIS)[0]


    def calculate_enemies_threat_weight(self, pos):
        weight_sum=0.00000001

        for disturb_id in self.qb_aircrafts_info_current[UnitType.DISTURB]:
            weight_sum = weight_sum + DISTURB_WEIGHT*1/calculate_2d_distance(pos,Dict_GetUnitPosByID(self.dict_qb, disturb_id, 2)[1])
        for A2A_id in self.qb_aircrafts_info_current[UnitType.A2A]:
            weight_sum = weight_sum + A2A_WEIGHT*1/calculate_2d_distance(pos,Dict_GetUnitPosByID(self.dict_qb, A2A_id, 2)[1])
        for A2G_id in self.qb_aircrafts_info_current[UnitType.A2G]:
            weight_sum = weight_sum + A2G_WEIGHT*1/calculate_2d_distance(pos,Dict_GetUnitPosByID(self.dict_qb, A2G_id, 2)[1])
        for AWACS_id in self.qb_aircrafts_info_current[UnitType.AWACS]:
            weight_sum = weight_sum + AWACS_WEIGHT*1/calculate_2d_distance(pos,Dict_GetUnitPosByID(self.dict_qb, AWACS_id, 2)[1])
        for UNAIRTAR_id in self.qb_aircrafts_info_current[UnitType.UNAIRTAR]:
            weight_sum = weight_sum + UNAIRTAR_WEIGHT*1/calculate_2d_distance(pos,Dict_GetUnitPosByID(self.dict_qb, UNAIRTAR_id, 2)[1])
        return weight_sum**SCALE_FACTOR

    def calcualte_defense_position(self, defense_target_pos, defense_distance, defenser_num, threat_distance):
        if defenser_num==0:
            return []

        #待细化
        enemy_center=np.array([0,0])
        enemyInfo = {} #id:[weight,angle]
        for disturb_id in self.qb_aircrafts_info_current[UnitType.DISTURB]:
            enemy_pos = Dict_GetUnitPosByID(self.dict_qb,disturb_id,2)[1]
            distance_vec = np.array(enemy_pos) - np.array(defense_target_pos)
            dis,angle=cartesianToPolar(distance_vec[0],distance_vec[1])
            if lengthofVector(distance_vec)<threat_distance:
                enemy_center = enemy_center + DISTURB_WEIGHT * distance_vec
                enemyInfo[disturb_id] = [DISTURB_WEIGHT,angle,distance_vec]
        for A2A_id in self.qb_aircrafts_info_current[UnitType.A2A]:
            enemy_pos = Dict_GetUnitPosByID(self.dict_qb,A2A_id,2)[1]
            distance_vec = np.array(enemy_pos) - np.array(defense_target_pos)
            dis,angle=cartesianToPolar(distance_vec[0],distance_vec[1])
            if lengthofVector(distance_vec)<threat_distance:
                enemy_center = enemy_center + A2A_WEIGHT * distance_vec
                enemyInfo[A2A_id] = [A2A_WEIGHT,angle,distance_vec]
        for A2G_id in self.qb_aircrafts_info_current[UnitType.A2G]:
            enemy_pos = Dict_GetUnitPosByID(self.dict_qb,A2G_id,2)[1]
            distance_vec = np.array(enemy_pos) - np.array(defense_target_pos)
            dis,angle=cartesianToPolar(distance_vec[0],distance_vec[1])
            if lengthofVector(distance_vec)<threat_distance:
                enemy_center = enemy_center + A2G_WEIGHT * distance_vec
                enemyInfo[A2G_id] = [A2G_WEIGHT,angle,distance_vec]

        startAngle = math.atan2(-enemy_center[1],-enemy_center[0])
        if len(enemyInfo)==0:
            return int(defenser_num) * [[defense_target_pos[0] + defense_distance * math.cos(0), defense_target_pos[1] + defense_distance * math.sin(0)]]
        return defenser_num * [[defense_target_pos[0] + defense_distance * math.cos(startAngle + math.pi), defense_target_pos[1] + defense_distance * math.sin(startAngle + math.pi)]]
        allWeight = 0
        for info in enemyInfo.values():
            info[1][1] = (info[1][1]-startAngle) - 2*math.pi*math.floor((info[1][1]-startAngle)/(2*math.pi))
            allWeight = allWeight + info[1][0]

        sortedEnemyInfo = sorted(enemyInfo.items(), key=lambda kv: (kv[1][1], ), reverse=0)
        assignments=[0]*len(sortedEnemyInfo)
        weightsAssigned = 0
        for idx,values in enumerate(sortedEnemyInfo):
            weightsAssigned = weightsAssigned+values[1][0]
            assignments[idx] = math.floor(defenser_num * weightsAssigned / (allWeight + 0.001))


    def ship_planning(self):
        cmd = []
        if self.curr_time<200:
            return cmd
        if self.north_ship_id is not None:
            if self.north_ship_defense_pos is not None:
                cmd.extend(self._ship_areapatrol(self.north_ship_id,[self.north_ship_defense_pos[0],self.north_ship_defense_pos[1],0]))
            elif self.south_ship_defense_pos is not None:
                cmd.extend(self._ship_areapatrol(self.north_ship_id,[self.south_ship_defense_pos[0],self.south_ship_defense_pos[1],0]))
        if self.south_ship_id is not None:
            if self.south_ship_defense_pos is not None:
                cmd.extend(self._ship_areapatrol(self.south_ship_id,[self.south_ship_defense_pos[0],self.south_ship_defense_pos[1],0]))
            elif self.north_ship_defense_pos is not None:
                cmd.extend(self._ship_areapatrol(self.south_ship_id,[self.north_ship_defense_pos[0],self.north_ship_defense_pos[1],0]))
        return cmd

        '''
        flag, ship_ids = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)
        R=80000.0
        Num=5
        SHIP_TEMP_POINTS = [[-1000,-R+(i+0.5)*(2*R)/Num,0] for i in range(Num)]
        for ship_id in ship_ids:
            ship_point = None
            if self.curr_time < 20:
                idx = int(Num*self.curr_time/100)
                if idx not in self.usedIdx:
                    self.usedIdx.append(idx)
                    ship_point = SHIP_TEMP_POINTS[idx]
            else:
                if self.count_ship_set < 2:
                    ship_point = SHIP_POINT
                    self.count_ship_set = self.count_ship_set + 1
            if ship_point is not None:
                cmd.extend(self._ship_movedeploy(ship_id, ship_point))
        return cmd'''

    # 20100917
    def fighters_planning(self):
        self.update_A2A_Missle_Manager_state()

        cmd = []
        #计算存活的指挥所和驱逐舰，准备设置战斗机掩护巡逻点
        self.ship_id = None
        self.north_cmd_id = None
        self.south_cmd_id = None
        self.north_ship_id = None
        self.south_ship_id = None
        currentPatrolPoss = []
        flag, ship_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)
        _flag, ship_Poss = Dict_GetUnitsPosByIDList(self.dict_units, ship_IDs, 2)

        flag, cmd_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.COMMAND)
        _flag,cmd_Poss = Dict_GetUnitsPosByIDList(self.dict_units, cmd_IDs,2)
        for idx,cmd_Pos in enumerate(cmd_Poss):
            if calculate_2d_distance(cmd_Pos,NEW_NORTH_COMMAND_POS[0:2])<calculate_2d_distance(cmd_Pos,NEW_SOUTH_COMMAND_POS[0:2]):
                self.north_cmd_id=cmd_IDs[idx]
            else:
                self.south_cmd_id=cmd_IDs[idx]

        if self.north_cmd_id is not None:
            currentPatrolPoss.append(Fighter_Near_North_CMD_Pos)
        if self.south_cmd_id is not None:
            currentPatrolPoss.append(Fighter_Near_South_CMD_Pos)

        for ship_idx,ship_Pos in enumerate(ship_Poss):
            if calculate_2d_distance(ship_Pos,NEW_NORTH_COMMAND_POS[0:2])<calculate_2d_distance(ship_Pos,NEW_SOUTH_COMMAND_POS[0:2]):
                self.north_ship_id=ship_IDs[ship_idx]
                currentPatrolPoss.append(Fighter_Near_North_Ship_Pos)
            else:
                self.south_ship_id=ship_IDs[ship_idx]
                currentPatrolPoss.append(Fighter_Near_South_Ship_Pos)

        if len(currentPatrolPoss)==0:
            return cmd
        Fighter_Patrol_Pos={}#{id:pos}
        flag, fighter_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.A2A)
        fighters_Command = dict()
        for fighter_ID in fighter_IDs:
            fighters_Command[fighter_ID] = FighterCommand(fighter_ID)

        fighter_ids_with_mount = [fighter_idx for fighter_idx in fighter_IDs if
                                 Dict_GetUnitMunNumByID(self.dict_units, fighter_idx)[1] > 0]
        fighter_ids_without_mount = [fighter_idx for fighter_idx in fighter_IDs if
                                     fighter_idx not in fighter_ids_with_mount]

        # 1 规避指令：没弹的战斗机遇敌（战斗机和驱逐舰）则避开, 有弹的战斗机则规避驱逐舰
        detected_red_ship_ids = []
        for red_ship_id in self.red_ship_ids:
            flag, pos = Dict_GetUnitPosByID(self.dict_qb, red_ship_id, 2)
            if flag:
                detected_red_ship_ids.append(red_ship_id)

        flag, red_air_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb, self.qb_aircrafts_info_current[UnitType.A2A], 2)
        flag, red_ship_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb, detected_red_ship_ids, 2)

        # 没弹的战斗机遇敌（战斗机和驱逐舰）则避开
        flag, blue_fighter_pos_list = Dict_GetUnitsPosByIDList(self.dict_units, fighter_ids_without_mount, 2)
        for blue_idx,blue_fighter_id in enumerate(fighter_ids_without_mount):
            escape_vector = np.array([0,0])
            for red_air_pos in red_air_pos_list:
                distance = ((blue_fighter_pos_list[blue_idx][0]-red_air_pos[0])**2+(blue_fighter_pos_list[blue_idx][1]-red_air_pos[1])**2)**0.5
                if distance < 110000:
                    escape_vector = escape_vector + (111000-distance)*(np.array(blue_fighter_pos_list[blue_idx]) - np.array(red_air_pos))
            for red_ship_pos in red_ship_pos_list:
                distance = ((blue_fighter_pos_list[blue_idx][0]-red_ship_pos[0])**2+(blue_fighter_pos_list[blue_idx][1]-red_ship_pos[1])**2)**0.5
                if distance < 150000:
                    escape_vector = escape_vector + (151000-distance)*(np.array(blue_fighter_pos_list[blue_idx]) - np.array(red_ship_pos))
            escape_vector_length = lengthofVector(escape_vector)
            if escape_vector_length>1:
                escape_target_point = np.array(blue_fighter_pos_list[blue_idx])+10000/escape_vector_length*escape_vector
                escape_target_point1 = np.array(
                    blue_fighter_pos_list[blue_idx]) + 20000 / escape_vector_length * escape_vector
                escape_lineparams = [
                    Point(escape_target_point[0], escape_target_point[1], 7500),
                    Point(escape_target_point1[0], escape_target_point1[1], 7500)]
                fighters_Command[blue_fighter_id].escape_cmd = self._aircraft_linepatrol(blue_fighter_id, 999/3.6, escape_lineparams)
        # 有弹的战斗机则规避驱逐舰
        flag, blue_fighter_pos_list = Dict_GetUnitsPosByIDList(self.dict_units, fighter_ids_with_mount, 2)
        for blue_idx,blue_fighter_id in enumerate(fighter_ids_with_mount):
            escape_vector = np.array([0,0])
            for red_ship_pos in red_ship_pos_list:
                distance = ((blue_fighter_pos_list[blue_idx][0]-red_ship_pos[0])**2+(blue_fighter_pos_list[blue_idx][1]-red_ship_pos[1])**2)**0.5
                if distance < 150000:
                    escape_vector = escape_vector + (151000-distance)*(np.array(blue_fighter_pos_list[blue_idx]) - np.array(red_ship_pos))
            escape_vector_length = lengthofVector(escape_vector)
            if escape_vector_length>1:
                escape_target_point = np.array(blue_fighter_pos_list[blue_idx])+10000/escape_vector_length*escape_vector
                escape_target_point1 = np.array(
                    blue_fighter_pos_list[blue_idx]) + 20000 / escape_vector_length * escape_vector
                escape_lineparams = [
                    Point(escape_target_point[0], escape_target_point[1], 7500),
                    Point(escape_target_point1[0], escape_target_point1[1], 7500)]
                fighters_Command[blue_fighter_id].escape_cmd = self._aircraft_linepatrol(blue_fighter_id, 999/3.6, escape_lineparams)

        # 2返航指令：没弹的回家
        for blue_idx,blue_fighter_id in enumerate(fighter_ids_without_mount):

            flag, fighter_pos = Dict_GetUnitPosByID(self.dict_units,blue_fighter_id,2)
            distance_to_airport = calculate_2d_distance(fighter_pos, AIRPORT_POS)
            if distance_to_airport<30000:
                fighters_Command[blue_fighter_id].return_base_cmd = self._returntobase(blue_fighter_id)
                fighters_Command[blue_fighter_id].escape_cmd = None
            else:
                fighter_MassArea_Params = [270, 3000, 3000, 999.9 / 3.6, 6000,7200]
                fighters_Command[blue_fighter_id].return_base_cmd = self._aircraft_areapatrol(blue_fighter_id, AIRPORT_POS,
                                                                                  fighter_MassArea_Params)

        # 3有弹能开火就开火,注意火力分配
        flag, blue_fighter_pos_list = Dict_GetUnitsPosByIDList(self.dict_units,fighter_ids_with_mount,2)
        flag, red_air_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb,self.red_air_ids_current,2)

        self_target_fire_dict=dict()
        for blue_idx,blue_fighter_id in enumerate(fighter_ids_with_mount):
            for red_idx,red_fighter_id in enumerate(self.red_air_ids_current):
                distance = ((blue_fighter_pos_list[blue_idx][0]-red_air_pos_list[red_idx][0])**2+(blue_fighter_pos_list[blue_idx][1]-red_air_pos_list[red_idx][1])**2)**0.5
                if red_fighter_id in self.qb_aircrafts_info_current[UnitType.DISTURB] and distance<135000:
                    self_target_fire_dict[(blue_fighter_id,red_fighter_id)]=distance
                if distance<110000:
                    self_target_fire_dict[(blue_fighter_id,red_fighter_id)]=distance
                elif red_fighter_id in self.qb_aircrafts_info_current[UnitType.A2G] and distance<120000:
                    self_target_fire_dict[(blue_fighter_id, red_fighter_id)] = distance
        sorted_fire_distance_items = sorted(self_target_fire_dict.items(), key=lambda kv: (kv[1], kv[0]), reverse=False)
        fighter_assigned = set()
        target__assigned = set()
        for (index,distance) in sorted_fire_distance_items:
            blue_id = index[0]
            red_id = index[1]
            if red_id in target__assigned or blue_id in fighter_assigned:
                break
            target_alive_Missle = self.A2A_Missle_Manager.getAliveMissle(red_id,self.curr_time)
            maxAttackMissle =2
            if red_id in self.qb_aircrafts_info_current[UnitType.A2G]:
                maxAttackMissle=1
            if target_alive_Missle<maxAttackMissle: #and Dict_GetUnitPosByID(self.dict_units,blue_id,2)[1][0] < -95000:
                fighter_assigned.add(blue_id)
                target__assigned.add(red_id)
                fighters_Command[blue_id].attack_fighter_cmd = (blue_id,red_id,distance/1000)
                if distance<100000:  #若敌战斗机过于接近，则优先发射导弹攻击指令，取消规避驱逐舰指令。
                    fighters_Command[blue_id].escape_cmd = None

        #4 平均分配巡逻区域
        for blue_idx, blue_fighter_id in enumerate(fighter_ids_with_mount):
            Fighter_MassArea_Params = [270, 10000, 10000, 1000 / 3.61, 7200]
            Fighter_MassArea_Params[0] = np.random.randint(0, 359)
            Fighter_MassArea_Params[1] = np.random.randint(3000, 10000)
            Fighter_MassArea_Params[2] = np.random.randint(3000, 10000)
            if blue_fighter_id in self.fighter_id_pos_dict:
                fighters_Command[blue_fighter_id].patrol_cmd = self._aircraft_areapatrol(blue_fighter_id,
                                                                                           [self.fighter_id_pos_dict[blue_fighter_id][0],
                                                                                            self.fighter_id_pos_dict[blue_fighter_id][1],
                                                                                            7000],
                                                                                           Fighter_MassArea_Params)

        '''
        avgNumber = len(fighter_ids_with_mount)//len(currentPatrolPoss)
        rest = len(fighter_ids_with_mount)%len(currentPatrolPoss)
        flag, cmd_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.COMMAND)
        offeset1=0
        if len(cmd_IDs)==1:
            offeset1=20000
        for currentPatrolPos in currentPatrolPoss:
            currNum = avgNumber+rest
            rest=0
            for i in range(currNum):
                distances = [calculate_2d_distance(currentPatrolPos[0:2],Dict_GetUnitPosByID(self.dict_units,fighterID,2)[1]) for fighterID in fighter_ids_with_mount]
                idx = np.argmin(distances)
                assignedfighterId = fighter_ids_with_mount[idx]
                fighter_ids_with_mount.remove(assignedfighterId)
                Fighter_MassArea_Params = [270, 10000, 10000, 1000 / 3.61, 7200]
                Fighter_MassArea_Params[0]=np.random.randint(0,359)
                Fighter_MassArea_Params[1]=np.random.randint(3000,10000)
                Fighter_MassArea_Params[2] = np.random.randint(3000, 10000)
                fighters_Command[assignedfighterId].patrol_cmd = self._aircraft_areapatrol(assignedfighterId,[currentPatrolPos[0]+offeset1,currentPatrolPos[1],currentPatrolPos[2]], Fighter_MassArea_Params)
        '''
        for fighter_ID in fighters_Command:
            if fighters_Command[fighter_ID].attack_fighter_cmd is not None:
                cmd.extend(self._airattack(fighters_Command[fighter_ID].attack_fighter_cmd[0],fighters_Command[fighter_ID].attack_fighter_cmd[1]))
                self.last_air_cmd_info[fighter_ID]=[fighters_Command[fighter_ID].attack_fighter_cmd[0],fighters_Command[fighter_ID].attack_fighter_cmd[1],fighters_Command[fighter_ID].attack_fighter_cmd[2],
                                                    Dict_GetUnitMunNumByID(self.dict_units, fighter_ID)[1]]#{id:[blue_fighter_id,red_air_id,fly_time,mount_num_when_cmd]
                continue
            if fighters_Command[fighter_ID].escape_cmd is not None:
                cmd.extend(fighters_Command[fighter_ID].escape_cmd)
                continue
            if fighters_Command[fighter_ID].return_base_cmd is not None:
                cmd.extend(fighters_Command[fighter_ID].return_base_cmd)
                continue
            if fighters_Command[fighter_ID].patrol_cmd is not None:
                cmd.extend(fighters_Command[fighter_ID].patrol_cmd)
                continue
        return cmd


    def bombers_planning_ns(self):
        cmd = []
        bomber_num = self.obs['airports'][0]['BOM']
        if bomber_num > 0 and self.curr_time>600:
            cmd.extend(self._takeoff_areapatrol(1, UnitType.A2G, Bomber_Mass_Point_Middle,
                                                     Bomber_MassArea_Params))
        # 缓存更新敌方船的位置和发现时间
        flag, red_ship_ids = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.SHIP, Side.RED.value)
        flag, unknown_ship_ids = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.SHIP, Side.UNKNOWN.value)
        print("直接发现的红方舰艇ID:",red_ship_ids)
        for unknown_ship_id in unknown_ship_ids:
            if unknown_ship_id in self.red_ship_info_cache_dict:
                red_ship_ids.append(unknown_ship_id)
        print("综合历史红方舰艇ID和不明舰艇的全部红方舰艇ID:", red_ship_ids)
        flag, red_ship_poss = Dict_GetUnitsPosByIDList(self.dict_qb, red_ship_ids, 2)
        for red_ship_id,red_ship_pos in zip(red_ship_ids,red_ship_poss):
            self.red_ship_info_cache_dict[red_ship_id] = [red_ship_pos[0],red_ship_pos[1],self.curr_time] # {id:[x,y,simTime]}
        # 计算轰炸机的攻击目标，离北岛近的驱逐舰为第一攻击目标，离南岛近的驱逐舰为第二攻击目标
        target_red_ship = None
        near_north_red_ship=None
        distances_north = [calculate_2d_distance(red_ship_pos, NORTH_COMMAND_POS) for red_ship_pos in red_ship_poss]
        if len(distances_north)>0:
            min_idx = np.argmin(distances_north)
            if distances_north[min_idx]<170000:
                near_north_red_ship = red_ship_ids[min_idx]
        near_south_red_ship = None
        distances_south = [calculate_2d_distance(red_ship_pos, SOUTH_COMMAND_POS) for red_ship_pos in red_ship_poss]
        if len(distances_south)>0:
            min_idx = np.argmin(distances_south)
            if distances_north[min_idx] < 170000:
                near_south_red_ship = red_ship_ids[min_idx]
        if near_north_red_ship is not None:
            target_red_ship = near_north_red_ship
        elif near_south_red_ship is not None:
            target_red_ship = near_south_red_ship

        flag, bomber_IDs = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.A2G)
        attack_complete_bombers = []
        for bomber_id in bomber_IDs:
            if bomber_id in self.last_bombers_attack:
                flag, curr_bomber_mount_num = Dict_GetUnitMunNumByID(self.dict_units,bomber_id)
                last_bomber_mount_num = self.last_bombers_attack[bomber_id][2]
                if last_bomber_mount_num > curr_bomber_mount_num:
                    attack_complete_bombers.append(bomber_id)
        for bomber_id in attack_complete_bombers:
            del self.last_bombers_attack[bomber_id]
        # 轰炸机规避、发射导弹、以及攻击航线规划
        bombers_command = dict()
        for bomber_ID in bomber_IDs:
            bombers_command[bomber_ID] = BomberCommand(bomber_ID)

        bomber_ids_with_mount = [bomber_id for bomber_id in bomber_IDs if
                                 Dict_GetUnitMunNumByID(self.dict_units, bomber_id)[1] > 0]
        bomber_ids_without_mount = [bomber_id for bomber_id in bomber_IDs if
                                     bomber_id not in bomber_ids_with_mount]

        # 1 规避指令：没弹的轰炸机遇敌（战斗机和驱逐舰）则避开, 有弹的轰炸机则规避战斗机
        detected_red_ship_ids = []
        for red_ship_id in self.red_ship_ids:
            flag, pos = Dict_GetUnitPosByID(self.dict_qb, red_ship_id, 2)
            if flag:
                detected_red_ship_ids.append(red_ship_id)
        flag, red_air_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb, self.qb_aircrafts_info_current[UnitType.A2A], 2)
        flag, red_ship_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb, detected_red_ship_ids, 2)

        # 没弹的轰炸机遇敌（战斗机和驱逐舰）则避开
        flag, blue_bomber_no_mount_pos_list = Dict_GetUnitsPosByIDList(self.dict_units, bomber_ids_without_mount, 2)
        for blue_idx,blue_bomber_id in enumerate(bomber_ids_without_mount):
            escape_vector = np.array([0,0])
            for red_air_pos in red_air_pos_list:
                distance = ((blue_bomber_no_mount_pos_list[blue_idx][0]-red_air_pos[0])**2+(blue_bomber_no_mount_pos_list[blue_idx][1]-red_air_pos[1])**2)**0.5
                if distance < 79000:
                    escape_vector = escape_vector + (80000-distance)*np.array(blue_bomber_no_mount_pos_list[blue_idx]) - np.array(red_air_pos)
            for red_ship_pos in red_ship_pos_list:
                distance = ((blue_bomber_no_mount_pos_list[blue_idx][0]-red_ship_pos[0])**2+(blue_bomber_no_mount_pos_list[blue_idx][1]-red_ship_pos[1])**2)**0.5
                if distance < 100000:
                    escape_vector = escape_vector + (101000-distance)*np.array(blue_bomber_no_mount_pos_list[blue_idx]) - np.array(red_ship_pos)
            escape_vector_length = lengthofVector(escape_vector)
            if escape_vector_length>1:
                escape_target_point = np.array(blue_bomber_no_mount_pos_list[blue_idx])+10000/escape_vector_length*escape_vector
                escape_target_point1 = np.array(
                    blue_bomber_no_mount_pos_list[blue_idx]) + 20000 / escape_vector_length * escape_vector
                escape_lineparams = [
                    Point(escape_target_point[0], escape_target_point[1], 7500),
                    Point(escape_target_point1[0], escape_target_point1[1], 7500)]
                bombers_command[blue_bomber_id].escape_cmd = self._aircraft_linepatrol(blue_bomber_id, 799/3.6, escape_lineparams)
        # 有弹的轰炸机则规避战斗机
        flag, blue_bomber_with_mount_pos_list = Dict_GetUnitsPosByIDList(self.dict_units, bomber_ids_with_mount, 2)
        for blue_idx,blue_bomber_id in enumerate(bomber_ids_with_mount):
            escape_vector = np.array([0,0])
            for red_air_pos in red_air_pos_list:
                distance = ((blue_bomber_with_mount_pos_list[blue_idx][0]-red_air_pos[0])**2+(blue_bomber_with_mount_pos_list[blue_idx][1]-red_air_pos[1])**2)**0.5
                if distance < 79000:
                    escape_vector = escape_vector + (80000-distance)*np.array(blue_bomber_with_mount_pos_list[blue_idx]) - np.array(red_air_pos)

            escape_vector_length = lengthofVector(escape_vector)
            if escape_vector_length>1:
                escape_target_point = np.array(blue_bomber_with_mount_pos_list[blue_idx])+10000/escape_vector_length*escape_vector
                escape_target_point1 = np.array(
                    blue_bomber_with_mount_pos_list[blue_idx]) + 20000 / escape_vector_length * escape_vector
                escape_lineparams = [
                    Point(escape_target_point[0], escape_target_point[1], 7500),
                    Point(escape_target_point1[0], escape_target_point1[1], 7500)]
                bombers_command[blue_bomber_id].escape_cmd = self._aircraft_linepatrol(blue_bomber_id, 799.9/3.6, escape_lineparams)

        # 2返航指令：没弹的回家
        for blue_bomber_id in bomber_ids_without_mount:
            flag, bomber_pos = Dict_GetUnitPosByID(self.dict_units,blue_bomber_id,2)
            distance_to_airport = calculate_2d_distance(bomber_pos,AIRPORT_POS)
            if distance_to_airport<20000:
                bombers_command[blue_bomber_id].return_base_cmd = self._returntobase(blue_bomber_id)
                bombers_command[blue_bomber_id].escape_cmd=None
            else:
                bomber_MassArea_Params = [270, 3000, 3000, 799.9 / 3.6, 6000]
                bombers_command[bomber_id].return_base_cmd = self._aircraft_areapatrol(bomber_id, SOUTH_COMMAND_POS,
                                                                                  bomber_MassArea_Params)



        # 3接近攻击目标则开火
        if target_red_ship is not None:
            flag, target_red_ship_pos  = Dict_GetUnitPosByID(self.dict_qb,target_red_ship,2)
            bomber_ids_with_mount.sort()
            flag, blue_bomber_pos_list = Dict_GetUnitsPosByIDList(self.dict_units,bomber_ids_with_mount,2)
            for idx,bomber_id in enumerate(bomber_ids_with_mount):
                distance = calculate_2d_distance(blue_bomber_pos_list[idx],target_red_ship_pos)
                if distance < 80200:
                    bombers_command[bomber_id].attack_ship_cmd=(bomber_id,target_red_ship,distance/1000)
                # 接近发射时，则发动自杀式攻击,不再规避敌战斗机
                if distance < 90000:
                    bombers_command[bomber_id].escape_cmd=None

        # 4 规划攻击目标的航线
        if target_red_ship is not None and len(bomber_IDs)>=3:
            if self.curr_time>600:
                flag, target_red_ship_pos  = Dict_GetUnitPosByID(self.dict_qb,target_red_ship,2)
                bomber_ids_with_mount.sort()
                flag, blue_bomber_pos_list = Dict_GetUnitsPosByIDList(self.dict_units,bomber_ids_with_mount,2)
                for idx,bomber_id in enumerate(bomber_ids_with_mount):
                    distance_vector = np.array(target_red_ship_pos) - np.array(blue_bomber_pos_list[idx])
                    target_speed = np.array([-10,0])
                    self_max_speed = 799.9/3.6
                    self_opt_speed = optimalSpeed(target_speed, self_max_speed, distance_vector)
                    self_opt_speed_length = lengthofVector(self_opt_speed)+0.1
                    self_opt_speed = self_max_speed/self_opt_speed_length*self_opt_speed
                    attack_point0 = np.array(
                        blue_bomber_pos_list[idx]) + 10000 / self_opt_speed_length * self_opt_speed
                    attack_point1 = np.array(
                        blue_bomber_pos_list[idx]) + 20000 / self_opt_speed_length * self_opt_speed
                    attack_lineparams = [Point(attack_point0[0], attack_point0[1], 5000+200*idx),
                        Point(attack_point1[0], attack_point1[1], 5000+200*idx)]
                    bombers_command[bomber_id].attack_path_cmd = self._aircraft_linepatrol(bomber_id, 799 / 3.6, attack_lineparams)


        # 5 规划巡逻航线
        for idx, bomber_id in enumerate(bomber_ids_with_mount):
            bomber_MassArea_Params = [270, 10000, 10000, 799 / 3.6, 6000]
            bomber_MassArea_Params[0] = np.random.randint(0, 359)
            bomber_MassArea_Params[1] = np.random.randint(3000, 5000)
            bomber_MassArea_Params[2] = np.random.randint(3000, 5000)
            currentPatrolPos=[-100000,30000,7000]
            if self.curr_time>600:
                currentPatrolPos = [-150000,-120000,7000]
            bombers_command[bomber_id].patrol_cmd = self._aircraft_areapatrol(bomber_id,currentPatrolPos,bomber_MassArea_Params)

        for bomber_id in bombers_command:
            if bombers_command[bomber_id].escape_cmd is not None:
                cmd.extend(bombers_command[bomber_id].escape_cmd)
                continue
            if bombers_command[bomber_id].attack_ship_cmd is not None:
                _,mount_num = Dict_GetUnitMunNumByID(self.dict_units,bomber_id)
                _,bomber_pos = Dict_GetUnitPosByID(self.dict_units,bomber_id,2)
                _,target_ship_pos = Dict_GetUnitPosByID(self.dict_qb,bombers_command[bomber_id].attack_ship_cmd[1],2)
                distance_vector = np.array(target_ship_pos) - np.array(bomber_pos)
                distance = lengthofVector(distance_vector)
                distanceRatio = int(100*min(1.0,(distance-200)/80000))
                angle = CalcAzByTwoPts([0,0],distance_vector)
                angle = int((angle+360)%360)

                if bomber_id not in self.last_bombers_attack:
                    attack_cmd = self._targethunt(bomber_id, bombers_command[bomber_id].attack_ship_cmd[1], angle, distanceRatio)
                    cmd.extend(attack_cmd)
                    self.last_bombers_attack[bomber_id] = [self.curr_time,bombers_command[bomber_id].attack_ship_cmd[1],mount_num]
                else:
                    if self.last_bombers_attack[bomber_id][1]==bombers_command[bomber_id].attack_ship_cmd[1]:
                        self.last_bombers_attack[bomber_id]=[self.curr_time,bombers_command[bomber_id].attack_ship_cmd[1],mount_num]
                    else:
                        if self.curr_time - self.last_bombers_attack[bomber_id][0]>60:
                            attack_cmd = self._targethunt(bomber_id, bombers_command[bomber_id].attack_ship_cmd[1], angle, distanceRatio)
                            cmd.extend(attack_cmd)
                            self.last_bombers_attack[bomber_id] = [self.curr_time,
                                                              bombers_command[bomber_id].attack_ship_cmd[1],mount_num]
                continue
            if bombers_command[bomber_id].attack_ship_cmd is None:
                if bomber_id in self.last_bombers_attack:
                    if self.curr_time - self.last_bombers_attack[bomber_id][0] < 120:
                        continue
            if bombers_command[bomber_id].return_base_cmd is not None:
                cmd.extend(bombers_command[bomber_id].return_base_cmd)
                continue
            if bombers_command[bomber_id].attack_path_cmd is not None:
                cmd.extend(bombers_command[bomber_id].attack_path_cmd)
                continue
            if bombers_command[bomber_id].patrol_cmd is not None:
                cmd.extend(bombers_command[bomber_id].patrol_cmd)
                continue
        return cmd


    def update_A2A_Missle_Manager_state(self):
        for fighter_id in self.last_air_cmd_info:
            if self.last_air_cmd_info[fighter_id] is not None:
                flag, fighter_mun_num = Dict_GetUnitMunNumByID(self.dict_units,fighter_id)
                if flag:
                    if fighter_mun_num<self.last_air_cmd_info[fighter_id][3]:
                        self.A2A_Missle_Manager.addItem(self.last_air_cmd_info[fighter_id][1], self.curr_time+self.last_air_cmd_info[fighter_id][2])
                        self.last_air_cmd_info[fighter_id][3] = fighter_mun_num
                else:
                    self.last_air_cmd_info[fighter_id] = None


    def update_red_ships_detect_info(self):

        if len(self.red_ship_ids) == 2:
            return
        flag, red_ship_ids = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.SHIP, Side.RED.value)

        for id in red_ship_ids:
            if id not in self.red_ship_ids:
                self.red_ship_ids.append(id)
                self.red_ship_ids.sort()

    def update_red_aircraft_detect_info(self):
        flag, red_A2A_aircrafts = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.A2A, Side.RED.value)

        flag, red_A2G_aircrafts = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.A2G, Side.RED.value)
        flag, red_AWACS_aircrafts = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.AWACS, Side.RED.value)
        flag, red_DISTURB_aircrafts = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.DISTURB, Side.RED.value)
        flag, red_UAV_aircrafts = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.UAV, Side.RED.value)
        red_UAV_aircrafts = []
        flag, UNAIRTAR_aircrafts = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.UNAIRTAR, Side.UNKNOWN.value)

        flag, civial_aircrafts = Dict_GetQBUnitsID_ByType(self.dict_qb, UnitType.CIVILAVIATION, 3)


        self.qb_aircrafts_info_current = {UnitType.A2A:set(red_A2A_aircrafts), UnitType.A2G:set(red_A2G_aircrafts), UnitType.AWACS:set(red_AWACS_aircrafts),
                                          UnitType.DISTURB:set(red_DISTURB_aircrafts), UnitType.UNAIRTAR:set(), UnitType.UAV:set(red_UAV_aircrafts)}

        for id_A2A in red_A2A_aircrafts:
            if id_A2A not in self.red_aircrafts_info_history or self.red_aircrafts_info_history[id_A2A] == UnitType.UNAIRTAR:
                self.red_aircrafts_info_history[id_A2A] = UnitType.A2A

        for id_A2G in red_A2G_aircrafts:
            if id_A2G not in self.red_aircrafts_info_history or self.red_aircrafts_info_history[id_A2G] == UnitType.UNAIRTAR:
                self.red_aircrafts_info_history[id_A2G] = UnitType.A2G

        for id_AWACS in red_AWACS_aircrafts:
            if id_AWACS not in self.red_aircrafts_info_history or self.red_aircrafts_info_history[id_AWACS] == UnitType.UNAIRTAR:
                self.red_aircrafts_info_history[id_AWACS] = UnitType.AWACS

        for id_DISTURB in red_DISTURB_aircrafts:
            if id_DISTURB not in self.red_aircrafts_info_history or self.red_aircrafts_info_history[id_DISTURB] == UnitType.UNAIRTAR:
                self.red_aircrafts_info_history[id_DISTURB] = UnitType.DISTURB
        for id_UAV in red_UAV_aircrafts:
            if id_UAV not in self.red_aircrafts_info_history or self.red_aircrafts_info_history[id_UAV] == UnitType.UNAIRTAR:
                self.red_aircrafts_info_history[id_UAV] = UnitType.UAV


        for id_UNAIRTAR in UNAIRTAR_aircrafts:
            pos_id_UNAIRTAR = Dict_GetUnitPosByID(self.dict_qb, id_UNAIRTAR, 2)[1]
            distance = calculate_2d_distance(pos_id_UNAIRTAR, [146700,-3000])
            if (distance<60000) and (id_UNAIRTAR not in self.red_aircrafts_info_history):
                self.red_aircrafts_info_history[id_UNAIRTAR] = UnitType.UNAIRTAR

        for id_civial in civial_aircrafts:
            self.green_aircrafts_info_history[id_civial] = UnitType.CIVILAVIATION


        for id_UNAIRTAR in UNAIRTAR_aircrafts:
            if id_UNAIRTAR in self.red_aircrafts_info_history:
                self.qb_aircrafts_info_current[self.red_aircrafts_info_history[id_UNAIRTAR]].add(id_UNAIRTAR)
            elif id_UNAIRTAR not in self.green_aircrafts_info_history:
                self.qb_aircrafts_info_current[UnitType.UNAIRTAR].add(id_UNAIRTAR)

        self.red_air_ids_current.clear()
        for idset in self.qb_aircrafts_info_current.values():
            self.red_air_ids_current = self.red_air_ids_current.union(idset)

    # add by lq
    def air_defense_planning2(self, sim_time):
        # print('current air targets')
        # print(self.red_air_ids_current)
        cmd_list = []
        assign_df =  self.air_defense_assign_info_history
        # columns = ['target_id', 'target_type', 'air_defense_id', 'air_defense_type', 'start_time', 'end_time']
        # judge idle channels
        # ad_df =  self.own_air_defense_df
        air_defense_ids = self.own_air_defense_df['air_defense_id'].tolist()

        # delete history info
        if len(assign_df) > 0:
            assign_df = assign_df.drop(assign_df[assign_df.end_time <= sim_time].index)

        # calculate idle channels
        for air_defense_id in air_defense_ids:
            # get air defense mun number and type
            _, mun_num = GetUnitMunNumByID(self.dict_units, air_defense_id)
            ad_type = self.own_air_defense_df[self.own_air_defense_df.air_defense_id == air_defense_id]['air_defense_type'].tolist()[0]

            # calculation occupied channels
            items = assign_df[(assign_df.air_defense_id == air_defense_id) & (assign_df.end_time > sim_time)]['air_defense_type'].tolist()


            if ad_type == UnitType.SHIP:
                self.own_air_defense_df.loc[self.own_air_defense_df['air_defense_id'] == air_defense_id, ['idle_channel_num']] = [min(mun_num, AirDefenseMaxChannel.SHIP_CHANNEL - len(items))]
            else:
                self.own_air_defense_df.loc[self.own_air_defense_df['air_defense_id'] == air_defense_id, ['idle_channel_num']] = [min(mun_num, AirDefenseMaxChannel.S2A_CHANNEL - len(items))]

        if np.sum(self.own_air_defense_df['idle_channel_num'].values) > 0:
            print('current air target:', self.red_air_ids_current)
            # filter targets
            if len(self.red_air_ids_current) > 0:
                # if len(assign_df)>0:
                #     print('aaa')

                new_targets = air_defense_filter_targets(self.dict_units, air_defense_ids, self.dict_qb, self.red_air_ids_current, assign_df, sim_time)

                if len(new_targets) > 0:
                    new_target_assign, self.own_air_defense_df = air_defense_assignment(self.own_air_defense_df , new_targets)
                    # print(len(new_target_assign))
                    # print(self.own_air_defense_df)

                    # form cmd
                    for index, row in new_target_assign.iterrows():
                        cmd_list.extend(self.air_defense_attack(row['air_defense_id'], row['air_defense_type'],
                                                                row['target_id']))
                    self.air_defense_assign_info_history = pd.concat([assign_df, new_target_assign], axis=0)

                    # print(self.air_defense_assign_info_history[['air_defense_id', 'target_id', 'start_time', 'end_time']])


        return cmd_list

    def air_defense_planning(self, sim_time):
        cmd_list = []
        assign_dict =  self.air_defense_assign_history_dict

        # delete history info
        if len(assign_dict) > 0:
            delete_keys = []
            for (k,v) in assign_dict.items():
                if v.end_time <=  sim_time:
                    delete_keys.append(k)
            for key in delete_keys:
                del assign_dict[key]


        # calculate idle channels
        # clear zeros
        for (k,v) in self.own_air_defense_dict.items():
            # v为防空火力单元信息
            v.UsedChannelNum = 0

        for (k,v) in assign_dict.items():
            # v为防空指派信息
            if v.air_defense_id not in self.own_air_defense_dict:
                continue

            air_defense_info = self.own_air_defense_dict[v.air_defense_id]

            if air_defense_info.Type == UnitType.SHIP:
                if air_defense_info.UsedChannelNum < AirDefenseMaxChannel.SHIP_CHANNEL:
                    air_defense_info.UsedChannelNum += 1
            else:
                if air_defense_info.UsedChannelNum < AirDefenseMaxChannel.S2A_CHANNEL:
                    air_defense_info.UsedChannelNum += 1

        for (k, v) in self.own_air_defense_dict.items():
            # v为防空火力单元信息
            # 获取当前的弹药量
            _, mun_num = Dict_GetUnitMunNumByID(self.dict_units, k)

            if v.Type == UnitType.SHIP:
                current_idle_channels_num = AirDefenseMaxChannel.SHIP_CHANNEL - v.UsedChannelNum
                v.IdleChannelNum = min(mun_num, current_idle_channels_num)
            else:
                current_idle_channels_num = AirDefenseMaxChannel.S2A_CHANNEL - v.UsedChannelNum
                v.IdleChannelNum = min(mun_num, current_idle_channels_num)

        # print('air_defense_info:')
        # for (k, v) in self.own_air_defense_dict.items():
        #     print('air_defense_id: %d type: %d idle_channel_num: %d used_channel_num:%d' % (k, v.Type, v.IdleChannelNum, v.UsedChannelNum))

        if len(self.red_air_ids_current) > 0:
            new_targets = dict_air_defense_filter_targets(self.dict_units, list(self.own_air_defense_dict.keys()), self.dict_qb, self.red_air_ids_current, assign_dict, sim_time)

            if len(new_targets) > 0:
                start = time.time()
                new_target_assign = dict_air_defense_assignment(self.own_air_defense_dict, new_targets, sim_time, False)
                end = time.time()
                print('Blue Side air_defense_assignment Running time: %s Seconds' % (end - start))

                # form cmd
                for (k, v) in new_target_assign.items():
                    _, air_defense_type = Dict_GetUnitTypeByID(self.dict_units, v.air_defense_id)
                    cmd_list.extend(self.air_defense_attack(v.air_defense_id, air_defense_type, k))

                # Merge
                self.air_defense_assign_history_dict.update(new_target_assign)

        return cmd_list

    def air_defense_planning4(self, sim_time):
        cmd_list = []
        red_fighter = []
        red_awacs = []
        red_jammer = []
        red_bomber = []
        red_uav = []
        red_unknown = []

        # for red_air_id in self.red_air_ids_current:
        #     _, target_type = Dict_GetUnitTypeByID(self.dict_qb, red_air_id)
        #
        #     if target_type == UnitType.A2A:
        #         red_fighter.append(red_air_id)
        #     elif target_type == UnitType.A2G:
        #         red_bomber.append(red_air_id)
        #     elif target_type == UnitType.AWACS:
        #         red_awacs.append(red_air_id)
        #     elif target_type == UnitType.DISTURB:
        #         red_jammer.append(red_air_id)
        #     elif target_type == UnitType.UAV:
        #         red_uav.append(red_air_id)
        #     else:
        #         print('unknown')

        for (k, v) in self.qb_aircrafts_info_current.items():
            if k == UnitType.A2A:
                red_fighter.extend(v)
            elif k == UnitType.A2G:
                red_bomber.extend(v)
            elif k == UnitType.AWACS:
                red_awacs.extend(v)
            elif k == UnitType.DISTURB:
                red_jammer.extend(v)
            elif k == UnitType.UAV:
                pass
                #red_uav.extend(v)
            elif k == UnitType.UNAIRTAR:
                red_unknown.extend(v)
            else:
                print('unknown')

        for (k, v) in self.own_air_defense_dict.items():
            num = 0
            delete_targets=[]
            attack_targets=[]
            if v.Type == UnitType.SHIP:
                for uav_id in red_uav:
                    break
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, uav_id, 2)
                    if dist <= 100000:
                        num += 1

                        attack_targets.append(uav_id)
                    if dist <=110000:
                        delete_targets.append(uav_id)

                for jammer_id in red_jammer:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, jammer_id, 2)
                    if dist <= 145000:
                        num += 1
                        attack_targets.append(jammer_id)
                    if dist <= 155000:
                        delete_targets.append(jammer_id)


                for awacs_id in red_awacs:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, awacs_id, 2)
                    if dist <= 100000:
                        num += 1
                        attack_targets.append(awacs_id)
                    if dist <= 110000:
                        delete_targets.append(awacs_id)

                for bommer_id in red_bomber:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, bommer_id, 2)
                    if dist <= 90000:
                        num += 1
                        attack_targets.append(bommer_id)
                    if dist <= 110000:
                        delete_targets.append(bommer_id)

                for unknown_id in red_unknown:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, unknown_id, 2)
                    if dist <= 80000:
                        num += 1
                        attack_targets.append(unknown_id)
                    if dist <= 110000:
                        delete_targets.append(unknown_id)



                for fighter_id in red_fighter:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, fighter_id, 2)
                    if dist <= 75000:
                        num += 1
                        attack_targets.append(fighter_id)
                    if dist <= 110000:
                        delete_targets.append(fighter_id)



            else:
                for uav_id in red_uav:
                    break
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, uav_id, 2)
                    if dist <= 110000:
                        num += 1

                        attack_targets.append(uav_id)
                    if dist <= 110000:
                        delete_targets.append(uav_id)

                for jammer_id in red_jammer:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, jammer_id, 2)
                    if dist <= 145000:
                        num += 1
                        attack_targets.append(jammer_id)
                    if dist <= 155000:
                        delete_targets.append(jammer_id)

                for awacs_id in red_awacs:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, awacs_id, 2)
                    if dist <= 100000:
                        num += 1
                        attack_targets.append(awacs_id)
                    if dist <= 110000:
                        delete_targets.append(awacs_id)

                for bommer_id in red_bomber:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, bommer_id, 2)
                    if dist <= 90000:
                        num += 1
                        attack_targets.append(bommer_id)
                    if dist <= 110000:
                        delete_targets.append(bommer_id)

                for unknown_id in red_unknown:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, unknown_id, 2)
                    if dist <= 80000:
                        num += 1
                        attack_targets.append(unknown_id)
                    if dist <= 110000:
                        delete_targets.append(unknown_id)

                for fighter_id in red_fighter:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, k, fighter_id, 2)
                    if dist <= 75000:
                        num += 1
                        attack_targets.append(fighter_id)
                    if dist <= 110000:
                        delete_targets.append(fighter_id)


            for attack_target in attack_targets:
                cmd_list.extend(self.air_defense_attack(k, v.Type, attack_target))

            # if v.OnOff == True and num > 0:
            #     v.LastOptStep = self.step_count + 2


        # for (k, v) in self.own_air_defense_dict.items():
        #     if self.step_count % 10 == 0:
        #         self._air_defense_power_on_off(k, v.Type, 0)
        #
        #     if (self.step_count+1) % 10 == 0:
        #         self._air_defense_power_on_off(k, v.Type, 1)
        #
        #     if (self.step_count+2) % 10 == 0:
        #         self._air_defense_power_on_off(k, v.Type, 1)

            # if v.LastOptStep == self.step_count:
            #     if v.OnOff == True:
            #         self._air_defense_power_on_off(k, v.Type, 0)
            #         v.OnOff = False
            #         v.LastOptStep = self.step_count + 1
            #     else:
            #         self._air_defense_power_on_off(k, v.Type, 1)
            #         v.OnOff = True
            #         v.LastOptStep = self.step_count + 2

        # for (k, v) in self.own_air_defense_dict.items():
        #     print('air_defense_id:%d, OnOff:%d' %(k, v.OnOff))

        return cmd_list

    def air_defense_init(self):
        _, s2a_id_list = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.S2A)

        _, ship_id_list = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)

        _, s2a_pos_list = Dict_GetUnitsPosByIDList(self.dict_units, s2a_id_list, 2)

        self.own_ship_id = ship_id_list[0]
        _, ship_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_ship_id)

        south_s2a_index = 0
        for i, s2a_pos in enumerate(s2a_pos_list):
            if s2a_pos[1] > 0:
                self.own_north_s2a_id = s2a_id_list[i]
                _, north_s2a_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_north_s2a_id)
                # print('pos:[-126629.25, 86387.625]')
                # print(s2a_pos)
            else:
                if south_s2a_index == 0:
                    self.own_south_s2a_first_id = s2a_id_list[i]
                    _, south_s2a_first_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_south_s2a_first_id)
                    south_s2a_index += 1
                # else:
                #     self.own_south_s2a_second_id = s2a_id_list[i]
                #     _, south_s2a_second_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_south_s2a_second_id)

        self.own_air_defense_dict[self.own_ship_id] = AirDefenseInfo(UnitType.SHIP, 4, 0)
        self.own_air_defense_dict[self.own_north_s2a_id] = AirDefenseInfo(UnitType.S2A, 3, 0)
        self.own_air_defense_dict[self.own_south_s2a_first_id] = AirDefenseInfo(UnitType.S2A, 3, 0)
        # self.own_air_defense_dict[self.own_south_s2a_second_id] = AirDefenseInfo(UnitType.S2A, 3, 0)

        return

    def air_defense_update(self):
        ship_flag, ship_type = Dict_GetUnitTypeByID(self.dict_units, self.own_ship_id)

        if not ship_flag and self.own_ship_id in self.own_air_defense_dict:
            del self.own_air_defense_dict[self.own_ship_id]

        north_s2a_flag, north_s2a_type = Dict_GetUnitTypeByID(self.dict_units, self.own_north_s2a_id)

        if not north_s2a_flag and self.own_north_s2a_id in self.own_air_defense_dict:
            del self.own_air_defense_dict[self.own_north_s2a_id]

        south_s2a_first_flag, south_s2a_first_type = Dict_GetUnitTypeByID(self.dict_units, self.own_south_s2a_first_id)

        if not south_s2a_first_flag and self.own_south_s2a_first_id in self.own_air_defense_dict:
            del self.own_air_defense_dict[self.own_south_s2a_first_id]

        # south_s2a_second_flag, south_s2a_second_type = Dict_GetUnitTypeByID(self.dict_units, self.own_south_s2a_second_id)
        #
        # if not south_s2a_second_flag and self.own_south_s2a_second_id in self.own_air_defense_dict:
        #     del self.own_air_defense_dict[self.own_south_s2a_second_id]

    def s2a_power_off(self, self_id):
        return [EnvCmd.make_ground_radarcontrol(self_id, 0)]

        # s2a

    def air_defense_attack(self, self_id, self_type, target_id):
        if self_type == UnitType.S2A:
            return [EnvCmd.make_ground_addtarget(self_id, target_id)]
        if self_type == UnitType.SHIP:
            return [EnvCmd.make_ship_addtarget(self_id, target_id)]

    def air_defense_attack_cancel(self, self_id, self_type, target_id):
        if self_type == UnitType.S2A:
            return [EnvCmd.make_ground_removetarget(self_id, target_id)]
        if self_type == UnitType.SHIP:
            return [EnvCmd.make_ship_removetarget(self_id, target_id)]

    def fire_control(self, self_id, target_id, fire_range):
        fire_flag = False
        fire_cmd = []
        _, dist = CalcDistByTwoUnitID(self.dict_units, self.dict_qb, self_id, target_id, 2)

        _, self_type = GetUnitTypeByID(self.dict_units, self_id)
        # print('target_id:%d' % target_id)
        # print('dist:%f' % dist)
        if dist <= fire_range:
            fire_cmd.extend(self._s2a_attack(self_id, self_type[0], target_id))
            fire_flag = True

        return fire_flag, fire_cmd

    def air_defense_init2(self):
        _, s2a_id_list = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.S2A)

        _, ship_id_list = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)

        _, s2a_pos_list = Dict_GetUnitsPosByIDList(self.dict_units, s2a_id_list, 2)

        self.own_ship_id = ship_id_list[0]
        _, ship_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_ship_id)

        south_s2a_index = 0
        for i, s2a_pos in enumerate(s2a_pos_list):
            if s2a_pos[1] > 0:
                self.own_north_s2a_id = s2a_id_list[i]
                _, north_s2a_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_north_s2a_id)
                # print('pos:[-126629.25, 86387.625]')
                # print(s2a_pos)
            else:
                if south_s2a_index == 0:
                    self.own_south_s2a_first_id = s2a_id_list[i]
                    _, south_s2a_first_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_south_s2a_first_id)
                    south_s2a_index += 1
                else:
                    self.own_south_s2a_second_id = s2a_id_list[i]
                    _, south_s2a_second_mun_num = Dict_GetUnitMunNumByID(self.dict_units, self.own_south_s2a_second_id)

        self.own_air_defense_df = self.own_air_defense_df.append(
            [{'air_defense_id': self.own_ship_id, 'air_defense_type': UnitType.SHIP,
              'idle_channel_num': 4, 'current_mun_num': ship_mun_num}], ignore_index=True)
        self.own_air_defense_df = self.own_air_defense_df.append(
            [{'air_defense_id': self.own_north_s2a_id, 'air_defense_type': UnitType.S2A,
              'idle_channel_num': 3, 'current_mun_num': north_s2a_mun_num}], ignore_index=True)
        self.own_air_defense_df = self.own_air_defense_df.append(
            [{'air_defense_id': self.own_south_s2a_first_id, 'air_defense_type': UnitType.S2A,
              'idle_channel_num': 3, 'current_mun_num': south_s2a_first_mun_num}], ignore_index=True)
        self.own_air_defense_df = self.own_air_defense_df.append(
            [{'air_defense_id': self.own_south_s2a_second_id, 'air_defense_type': UnitType.S2A,
              'idle_channel_num': 3, 'current_mun_num': south_s2a_second_mun_num}], ignore_index=True)

        return

    def air_defense_update2(self):
        ship_flag, ship_type = Dict_GetUnitTypeByID(self.dict_units, self.own_ship_id)

        if not ship_flag:
            self.own_air_defense_df = self.own_air_defense_df.drop(self.own_air_defense_df[self.own_air_defense_df.air_defense_id == self.own_ship_id].index)

        north_s2a_flag, north_s2a_type = Dict_GetUnitTypeByID(self.dict_units, self.own_north_s2a_id)

        if not north_s2a_flag:
            self.own_air_defense_df = self.own_air_defense_df.drop(self.own_air_defense_df[self.own_air_defense_df.air_defense_id == self.own_north_s2a_id].index)

        south_s2a_first_flag, south_s2a_first_type = Dict_GetUnitTypeByID(self.dict_units, self.own_south_s2a_first_id)

        if not south_s2a_first_flag:
            self.own_air_defense_df = self.own_air_defense_df.drop(self.own_air_defense_df[self.own_air_defense_df.air_defense_id == self.own_south_s2a_first_id].index)


        south_s2a_second_flag, south_s2a_second_type = Dict_GetUnitTypeByID(self.dict_units, self.own_south_s2a_second_id)

        if not south_s2a_second_flag:
            self.own_air_defense_df = self.own_air_defense_df.drop(self.own_air_defense_df[self.own_air_defense_df.air_defense_id == self.own_south_s2a_second_id].index)

    def bomber_planning(self, obs):
        return []
        cmd = []
        # 获取轰炸机id
        flag, bomber_ids = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.A2G)
        bomber_ids_with_mount = [bomber_idx for bomber_idx in bomber_ids if
                                 Dict_GetUnitMunNumByID(self.dict_units, bomber_idx)[1] > 0]
        bomber_ids_without_mount = [bomber_idx for bomber_idx in bomber_ids if bomber_idx not in bomber_ids_with_mount]
        # bombers without mounts should return to airport 没弹药的回家
        for bomber_idx in bomber_ids_without_mount:
            cmd.extend(self._returntobase(bomber_idx))
        # get red ships detected now 获取敌方护卫舰信息
        detected_red_ship_ids = []
        for red_ship_id in self.red_ship_ids:
            flag, pos = Dict_GetUnitPosByID(self.dict_qb, red_ship_id, 2)
            if flag:
                detected_red_ship_ids.append(red_ship_id)
        #
        if len(detected_red_ship_ids) == 0 and self.curr_time >= 2400:
            for bom_id in bomber_ids_with_mount:
                cmd.extend(self._returntobase(bom_id))

        bomber_num_air = obs['airports'][0]['BOM']
        if len(detected_red_ship_ids) > 0 and bomber_num_air > 0:
            cmd.extend(self._takeoff_areapatrol(bomber_num_air, UnitType.A2G, Bomber_Mass_Point_Middle,
                                                Bomber_MassArea_Params))

        # flag, bomber_poss = Dict_GetUnitsPosByIDList(self.dict_units, bomber_ids_with_mount, 2)
        bomber_num = len(bomber_ids_with_mount)
        # if only one red ship detected, idle bombers should attack it
        # print(detected_red_ship_ids,'detected_red_ship_idsdetected_red_ship_idsdetected_red_ship_idsdetected_red_ship_idsdetected_red_ship_ids')
        if len(detected_red_ship_ids) == 1:
            target_ship_id = detected_red_ship_ids[0]
            if self.dict_qb[target_ship_id].Pos_X < 750000:
                for bomber_id in bomber_ids_with_mount:
                    # print(self.not_assigned(bomber_id),self.assigned_complete(bomber_id),'yuding-+-+-+-+-+-+-+--++-+-+-+-++-+-+--++-+++-+-+-+-+--+-++-+-+')
                    # if self.not_assigned(bomber_id) or self.assigned_complete(bomber_id):
                    flag, az = Dict_CalcAzByTwoUnitID(self.dict_units, self.dict_qb, bomber_id, target_ship_id, 2)
                    if az % 10 < 5:
                        az = az // 10 * 10
                    else:
                        az = (az // 10 + 1) * 10
                    print(az,
                          'az---------------------------------------------------------------------------------------------')
                    cmd.extend(self._targethunt(bomber_id, target_ship_id, az, 90))
                    attack_assignment = AttackAssignment()
                    attack_assignment.attack_assignment_state = AttackAssignmentState.ASSIGNMENT_WHEN_ONE_RED_SHIP
                    attack_assignment.mount_num = Dict_GetUnitMunNumByID(self.dict_units, bomber_id)[1]
                    self.bomber_attack_assignment_dict[bomber_id] = attack_assignment
        # if all two red ship detected, bombers should reassign equally to attack them
        if len(detected_red_ship_ids) == 2:
            group_one_num = bomber_num // 2
            # group_two_num = bomber_num - group_one_num

            target_ship_first = detected_red_ship_ids[0]
            target_ship_second = detected_red_ship_ids[1]
            if self.dict_qb[target_ship_first].Pos_X < 350000 and self.dict_qb[target_ship_second].Pos_X < 350000:
                _, target_ship_first_pos = Dict_GetUnitPosByID(self.dict_qb, target_ship_first, 2)
                target_ship_id = detected_red_ship_ids[0]
                # 判断两艘舰艇哪个离得更近
                dic = {}
                for i in range(2):
                    print(i, detected_red_ship_ids[i])
                    flag, pos = Dict_GetUnitPosByID(self.dict_qb, detected_red_ship_ids[i], 2)
                    print(pos)
                    dis = calculate_2d_distance(pos, [-131154, -87888])
                    print(SOUTH_COMMAND_POS)
                    dic[detected_red_ship_ids[i]] = dis
                print(dic)
                if dic[detected_red_ship_ids[0]] < dic[detected_red_ship_ids[1]]:
                    target_ship_id = detected_red_ship_ids[0]
                    for idx in range(bomber_num):
                        bomber_id = bomber_ids_with_mount[idx]
                        flag, az = Dict_CalcAzByTwoUnitID(self.dict_units, self.dict_qb, bomber_id, target_ship_id, 2)
                        if az % 10 < 3:
                            az = az // 10 * 10
                        elif az % 10 < 8:
                            az = az // 10 * 10 + 5
                        else:
                            az = (az // 10 + 1) * 10
                        cmd.extend(self._targethunt(bomber_id, target_ship_id, az, 90))
                        attack_assignment = AttackAssignment()
                        attack_assignment.attack_assignment_state = AttackAssignmentState.ASSIGNMENT_WHEN_TWO_RED_SHIP
                        attack_assignment.mount_num = Dict_GetUnitMunNumByID(self.dict_units, bomber_id)[1]
                        self.bomber_attack_assignment_dict[bomber_id] = attack_assignment
                elif dic[detected_red_ship_ids[0]] == dic[detected_red_ship_ids[1]]:
                    for idx in range(bomber_num):
                        if idx >= group_one_num:
                            target_ship_id = detected_red_ship_ids[1]

                        bomber_id = bomber_ids_with_mount[idx]
                        flag, az = Dict_CalcAzByTwoUnitID(self.dict_units, self.dict_qb, bomber_id, target_ship_id, 2)
                        if az % 10 < 5:
                            az = az // 10 * 10
                        elif az % 10 < 8:
                            az = az // 10 * 10 + 5
                        else:
                            az = (az // 10 + 1) * 10
                        cmd.extend(self._targethunt(bomber_id, target_ship_id, az, 90))
                        attack_assignment = AttackAssignment()
                        attack_assignment.attack_assignment_state = AttackAssignmentState.ASSIGNMENT_WHEN_TWO_RED_SHIP
                        attack_assignment.mount_num = Dict_GetUnitMunNumByID(self.dict_units, bomber_id)[1]
                        self.bomber_attack_assignment_dict[bomber_id] = attack_assignment
                else:
                    target_ship_id = detected_red_ship_ids[1]
                    for idx in range(bomber_num):
                        bomber_id = bomber_ids_with_mount[idx]
                        flag, az = Dict_CalcAzByTwoUnitID(self.dict_units, self.dict_qb, bomber_id, target_ship_id, 2)
                        if az % 10 < 5:
                            az = az // 10 * 10
                        elif az % 10 < 8:
                            az = az // 10 * 10 + 5
                        else:
                            az = (az // 10 + 1) * 10
                        cmd.extend(self._targethunt(bomber_id, target_ship_id, az, 90))
                        attack_assignment = AttackAssignment()
                        attack_assignment.attack_assignment_state = AttackAssignmentState.ASSIGNMENT_WHEN_TWO_RED_SHIP
                        attack_assignment.mount_num = Dict_GetUnitMunNumByID(self.dict_units, bomber_id)[1]
                        self.bomber_attack_assignment_dict[bomber_id] = attack_assignment
        return cmd
    def not_assigned(self, bomber_id):
        if self.bomber_attack_assignment_dict.get(bomber_id, None) is None:
            return True
        return False

    # 09-14
    def assigned_complete(self, bomber_id):
        if not self.not_assigned(bomber_id):
            attack_assignment = self.bomber_attack_assignment_dict[bomber_id]
            if Dict_GetUnitMunNumByID(self.dict_units, bomber_id)[1] <= attack_assignment.mount_num:
                return True
        return False

        # 逃逸命令 9-16

    def awacs_planning(self):
        cmd = []

        # 预警机遇敌（战斗机和驱逐舰）则避开
        # 预警机位置
        awacs_flag, awacs_ID = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.AWACS)
        if awacs_flag:
            flag, awacs_pos = Dict_GetUnitPosByID(self.dict_units, awacs_ID[0], 2)
            # 红方巡航舰的ID列表和 位置列表
            detected_red_ship_ids = []
            red_ship_pos_list = []
            for red_ship_id in self.red_ship_ids:
                flag, pos = Dict_GetUnitPosByID(self.dict_qb, red_ship_id, 2)
                if flag:
                    detected_red_ship_ids.append(red_ship_id)
                    red_ship_pos_list.append(pos)

            # 红方歼击机位置列表
            flag, red_air_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb, self.qb_aircrafts_info_current[UnitType.A2A],
                                                              2)
            flag, red_disturb_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb, self.qb_aircrafts_info_current[UnitType.DISTURB],
                                                                  2)
            # flag, red_ship_pos_list = Dict_GetUnitsPosByIDList(self.dict_qb, detected_red_ship_ids, 2)

            # 逃逸矢量
            escape_vector = np.array([0, 0])
            for red_air_pos in red_air_pos_list:
                distance = ((awacs_pos[0] - red_air_pos[0]) ** 2 + (awacs_pos[1] - red_air_pos[1]) ** 2) ** 0.5
                if distance <= 110000:
                    escape_vector = escape_vector + np.array(awacs_pos) - np.array(red_air_pos)
            for red_ship_pos in red_ship_pos_list:
                distance = ((awacs_pos[0] - red_ship_pos[0]) ** 2 + (awacs_pos[1] - red_ship_pos[1]) ** 2) ** 0.5
                if distance <= 105000:
                    escape_vector = escape_vector + np.array(awacs_pos) - np.array(red_ship_pos)
            escape_vector_length = lengthofVector(escape_vector)
            for red_disturb_pos in red_disturb_pos_list:
                distance = ((awacs_pos[0] - red_disturb_pos[0]) ** 2 + (awacs_pos[1] - red_disturb_pos[1]) ** 2) ** 0.5
                if distance <= 130000:
                    escape_vector = escape_vector + np.array(awacs_pos) - np.array(red_disturb_pos)
            escape_vector_length = lengthofVector(escape_vector)
            if escape_vector_length > 1:
                escape_target_point = np.array(awacs_pos) + 10000 / escape_vector_length * escape_vector
                escape_target_point1 = np.array(awacs_pos) + 20000 / escape_vector_length * escape_vector
                escape_lineparams = [
                    Point(escape_target_point[0], escape_target_point[1], 7500),
                    Point(escape_target_point1[0], escape_target_point1[1], 7500)]
                cmd.extend(self._aircraft_linepatrol(awacs_ID[0], 800 / 3.61, escape_lineparams))

            # 预警机有逃跑命令先逃跑，没有逃跑命令继续执行巡航命令。
            if len(cmd) == 0:

                distance1 = calculate_2d_distance(awacs_pos, AWACS_POS1)
                if distance1 >= 7500 and 0 <= self.curr_time <= 600:
                    cmd.extend(self._awacs_patrol(awacs_ID[0], AWACS_POS1, AWACS_PATROL_PARAMS))

                distance2 = calculate_2d_distance(awacs_pos, AWACS_POS2)
                if distance2 >= 7500 and 600 < self.curr_time <= 1800:
                    cmd.extend(self._awacs_patrol(awacs_ID[0], AWACS_POS2, AWACS_PATROL_PARAMS))

                distance3 = calculate_2d_distance(awacs_pos, AWACS_POS3)
                if distance3 >= 7500 and self.curr_time > 1800:
                    cmd.extend(self._awacs_patrol(awacs_ID[0], AWACS_POS3, AWACS_PATROL_PARAMS))
            else:
                cmd.extend(cmd)
                print('==================================   tao tao tao tao tao')
            return cmd

        return cmd

    def air_defense_deploy(self):
        cmd_list = []

        flag, ship_ids = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)
        #
        self.own_first_ship_id = ship_ids[0]
        self.own_second_ship_id = ship_ids[1]
        # self.own_third_ship_id = ship_ids[2]
        cmd_list.extend(self._ship_movedeploy(ship_ids[0], FIRST_SHIP_POINT))
        cmd_list.extend(self._ship_movedeploy(ship_ids[1], SECOND_SHIP_POINT))
        # cmd_list.extend(self._ship_movedeploy(ship_ids[2], FIRST_SHIP_POINT))

        flag, s2a_ids = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.S2A)
        self.own_first_s2a_id = s2a_ids[0]
        self.own_second_s2a_id = s2a_ids[1]
        self.own_third_s2a_id = s2a_ids[2]
        # self.own_fourth_s2a_id = s2a_ids[3]
        cmd_list.extend(self._s2a_movedeploy(s2a_ids[0], NORTH_FIRST_S2A_POS))
        cmd_list.extend(self._s2a_movedeploy(s2a_ids[1], NORTH_SECOND_S2A_POS))
        cmd_list.extend(self._s2a_movedeploy(s2a_ids[2], SOUTH_FIRST_S2A_POS))
        # cmd_list.extend(self._s2a_movedeploy(s2a_ids[2], SOUTH_SECOND_S2A_POS))

        return cmd_list

    # 航线巡逻[wt增加]
    @staticmethod
    def _aircraft_areapatrol(aircraft_id, areapatrol_point, areapatrol_params):
        return [EnvCmd.make_areapatrol(aircraft_id, *areapatrol_point, *areapatrol_params)]

    # 航线巡逻[wt增加]
    @staticmethod
    def _aircraft_linepatrol(aircraft_id, patrol_spd, line_params):
        return [EnvCmd.make_linepatrol(aircraft_id, patrol_spd, 1, 'line', line_params)]

    # 无人机出击
    @staticmethod
    def _uav_areapatrol(uav_id, uav_point, uav_params):
        return [EnvCmd.make_uav_areapatrol(uav_id, *uav_point, *uav_params)]

    # 预警机出击
    @staticmethod
    def _awacs_patrol(self_id, AWACS_PATROL_POINT, AWACS_PATROL_PARAMS):
        return [EnvCmd.make_awcs_areapatrol(self_id, *AWACS_PATROL_POINT, *AWACS_PATROL_PARAMS)]

    # 预警机航线巡逻侦察(逻辑有问题,慎用)
    @staticmethod
    def _awcs_linepatrol(self, self_id, speed, line_params):
        return [EnvCmd.make_awcs_linepatrol(self_id, speed, 0, 'line', line_params)]

    # 预警机护航
    def _awacs_escort(self, awacs_team_id):
        return [EnvCmd.make_takeoff_protect(RED_AIRPORT_ID, 2, awacs_team_id, 0, 100, 250)]

    # 干扰机进行区域干扰
    def _disturb_patrol(self, disturb_team_id, patrol_point, patrol_params):
        return [EnvCmd.make_disturb_areapatrol(disturb_team_id, *patrol_point, *patrol_params)]

    # 干扰机进行航线干扰
    def _disturb_linepatrol(self, self_id, point_list):
        return [EnvCmd.make_disturb_linepatrol(self_id, 160, 0, 'line', point_list)]

    # 轰炸机起飞突击
    @staticmethod
    def _takeoff_areahunt(num, area_hunt_point):
        return [EnvCmd.make_takeoff_areahunt(RED_AIRPORT_ID, num, 270, 80, *area_hunt_point, *[270, 1000, 1000, 160])]

    # 干扰机护航
    def _disturb_escort(self, disturb_team_id):
        return [EnvCmd.make_takeoff_protect(RED_AIRPORT_ID, 2, disturb_team_id, 1, 100, 250)]

    # 轰炸机护航
    def _A2G_escort(self, a2g_team_id):
        return [EnvCmd.make_takeoff_protect(RED_AIRPORT_ID, 2, a2g_team_id, 1, 100, 250)]

    # 起飞区域巡逻
    @staticmethod
    def _takeoff_areapatrol(num, lx, patrol_point, patrol_params):
        # patrol_params为5个参数
        return [EnvCmd.make_takeoff_areapatrol(BLUE_AIRPORT_ID, num, lx, *patrol_point, *patrol_params)]

    @staticmethod
    def _airattack(unit_id, target_id):
        return [EnvCmd.make_airattack(unit_id, target_id, 0)]

    # 区域巡逻
    @staticmethod
    def _areapatrol(unit_id, patrol_point, patrol_params):
        # patrol_params为6个参数
        return [EnvCmd.make_areapatrol(unit_id, *patrol_point, *patrol_params)]

    # 返航
    @staticmethod
    def _returntobase(unit_id):
        return [EnvCmd.make_returntobase(unit_id, 20001)]

    # 轰炸机目标突击
    @staticmethod
    def _targethunt(self_id, target_id, a2saz, a2srange):
        return [EnvCmd.make_targethunt(self_id, target_id, a2saz, a2srange)]


    # 护卫舰初始化部署
    def _ship_movedeploy(self, self_id, point):
        return [EnvCmd.make_ship_movedeploy(self_id, *point, 0, 1)]

    def _s2a_movedeploy(self, self_id, point):
        return [EnvCmd.make_ground_movedeploy(self_id, *point, 0, 1)]

    # 护卫舰区域巡逻
    def _ship_areapatrol(self, self_id, point):
        SHIP_PATROL_PARAMS_0[0] = np.random.randint(0, 359)
        return [EnvCmd.make_ship_areapatrol(self_id, *point, *SHIP_PATROL_PARAMS_0)]

    def _air_defense_power_on_off(self, self_id, self_type, on_off):
        if self_type == UnitType.S2A:
            return [EnvCmd.make_ground_radarcontrol(self_id, on_off)]
        if self_type == UnitType.SHIP:
            return [EnvCmd.make_ship_radarcontrol(self_id, on_off)]
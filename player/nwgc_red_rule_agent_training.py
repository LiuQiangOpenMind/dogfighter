from enum import Enum
import math, json
from collections import defaultdict
#from queue import PriorityQueue as PriQue
import pandas as pd
import time
import numpy as np

from .agent import Agent
from env.env_cmd import EnvCmd
from env.env_def import UnitType, UnitStatus, RED_AIRPORT_ID
from .MinMaxWeigthMatching import *
from .agent_util import *
from .agent_util_modifiy import  *
from .MinMaxCluster_util import maxmin_distance_cluster


AIR_PATROL_HEIGHT = 8000
A2G_PATROL_HEIGHT = 7000
AWACS_PATROL_HEIGHT = 7500
DISTURB_PATROL_HEIGHT = 8500

# 空战测试区域
A2A_FIGHTAREA_POINT = [0, -80000, 8000]
# dir, len, wid, speed, time, mode:0:air/1:surface/2:both
A2A_FIGHTAREA_PARAMS = [270, 20000, 20000, 250, 8000]
A2A_FIGHTAREA_POINT2 = [25 , 0 , 8000]
# dir, len, wid, speed, time, mode:0:air/1:surface/2:both
A2A_FIGHTAREA_PARAMS2 = [270, 20000, 20000, 250, 8000]

A2S_YANGDONG_POINT = [-79533,87664,8000]
A2S_YANGDONGAREA_PARAMS = [270, 10000, 10000, 300, 8000]

# 护卫舰
# 北部巡逻阵位
SHIP_POINT1 = [120000.5, 184000.0, 0]
# 中部巡逻阵位
# SHIP_POINT2 = [110000, -20000, 0]
SHIP_POINT2 = [150000, -85000, 0]
# 南部巡逻阵位
SHIP_POINT3 = [150000, -85000, 0]
SHIP_PATROL_PARAMS_0 = [270, 5000, 5000, 19.99, 3600, 0]

# 无人机
# 北部侦察阵位(-105000, 65000)--距离北岛约35km, 突击北岛后可快速确认战果.
UAV_POINT1 = [105000, 85000, 5000]
# 中部待命阵位(-125000, 0)--距离敌南/北岛距离均约88km, 距离己方北部干扰阵位约97km
UAV_POINT2 = [125000, 0, 5000]
# 中北侦察阵位(-125000, 45000)--北岛南侧约47km, 突击北岛后快速确认战果(备选阵位)
UAV_POINT3 = [-125000, 45000, 5000]
# 中南侦察阵位(-125000, -45000)--南岛北侧约47km, 突击南岛后快速确认战果
UAV_POINT4 = [125000, -5000, 5000]
# 南部待命阵位(-35000, -85000)--距离南岛约95km(敌地防视线内打击范围外), 距己北部干扰阵位约140km, 起吸引注意力作用
UAV_POINT5 = [-35000, -85000, 5000]
# 南部规避阵位(-35000, -55000)--向北部干扰阵位方向撤退30km
UAV_POINT6 = [-35000, -55000, 5000]
# 南部侦察阵位(-95000, -85000)--南岛东侧约37km, 突击南岛后快速确认战果
UAV_POINT7 = [-95000, -85000, 5000]
UAV_PATROL_PARAMS = [270, 20000, 20000, 80, 7200]
UAV_PATROL_PARAMS_0 = [270, 20000, 20000, 80, 7200, 0]
UAV_PATROL_PARAMS_1 = [270, 20000, 20000, 80, 7200, 1]

# 预警机
# 预警机北部待命阵位
AWACS_NORTHPATROL_POINT = [-10000, 60000, AWACS_PATROL_HEIGHT]
# 预警机南部规避阵位
AWACS_SOUTH_POINT = [55000, -30000, AWACS_PATROL_HEIGHT]
# 预警机北部规避阵位
AWACS_NORTH_POINT = [55000, 30000, AWACS_PATROL_HEIGHT]
# dir, len, wid, speed, time, mode:0:air/1:surface/2:both
AWACS_PATROL_PARAMS = [270, 10000, 10000, 160, 7200, 2]

AREA_HUNT_PARAMS = [270, 1000, 1000]

Red_Airport_Pos = [146228.0,-1965.0]
#空中编队集结点
Fighter_Mass_Point1 = [140000,10000,8000]
Fighter_Mass_Point2 = [160000,10000,8000]
Awacs_Mass_Point = [150000, 10000, 8000]
#空中集结区参数 dir, len, wid, speed, time, mode:0:air/1:surface/2:both,;'oded
Awacs_MassArea_Params = [270, 20000, 20000, 160, 7200, 1]
Fighter_MassArea_Params = [270, 20000, 20000, 160, 7200]

Bomer_Standby_Point  = [6402, 137667, 6000]
Bomer_Mass_Point     = [140000,20000,6000]
Bomer_Patrol_Params  = [270, 20000, 20000, 200, 7200]
Bomer_FormationPos = [[(i*(-15)), 20000] for i in range(0,20)]

#预警机位置数据-相对干扰机航向的方位和距离
Awacs_FormationPos = [[90,1000]]

Fighter_Max_Speed = 300
Fighter_min_Speed = 200

NORTH_COMMAND_POS = [-129532, 87667]
SOUTH_COMMAND_POS = [-131154,-87888]

NORTH_NAVLINE_POS  = [5000, 87667]
CENTER_NAVLINE_POS = [4000,-87888]

A2S_AttackDistance = 110000
A2S_MAXAttackRange = 115000

FORMATION_TO_A2STARGETDIST = 155000

BomerA2GNumInSametime = 6

class Point():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class RedAgentState(Enum):

    # 起飞全部结束
    END_TAKEOFF = 12

    ATTACK_STAGE                  = 20
    NORTHLINE_NAVIGATION_STAGE    = 21

    NORTHLINE_ATTACKSHIP_STAGE    = 30
    NORTHLINE_ATTACKSHIP_NAVSTAGE = 31
    NORTHLINE_ATTACKSHIP_TASKASSIGN_STAGE  = 32
    NORTHLINE_ATTACKSHIP_OCCUPYA2SPOSITIONS_STAGE = 33
    NORTHLINE_ATTACKSHIP_ATTACKING_STAGE = 34
    NORTHCOMMAND_ATTACKSHIP_ATTACKEND = 35

    COMMAND_ATTACK_BEGIN = 50
    COMMAND_ATTACK_NAVSTAGE = 52
    COMMAND_ATTACK_MISSIONASSIGN = 53
    COMMAND_ATTACK_OCCUPYA2GCPOS_STAGE = 54
    COMMAND_ATTACK_ATTACKING_STAGE = 55
    COMMAND_ATTACK_CMDEXCUTED = 56

    COMMAND_ATTACK_MISSONCOMPLETED = 100

class RedAgentOperationStage:
    Mass_Force   = 1
    Formation_Navaigation = 2
    Attack_Stage = 3
    Attack_Destroyer_Stage = 4

class NwgcRedAgentTraining(Agent):
    def __init__(self, name, config, **kwargs):
        super().__init__(name, config['side'])

        self._init()

    def _init(self):

        self.team_id_dic = {}
        self.Task = {}

        self.opration_stage = RedAgentOperationStage.Mass_Force
        self.Form_CurrNavPos = Awacs_Mass_Point
        self.agent_state = 0
        self.a2a_intercept_list = []
        self.formation_posassign_list = []
        self.awacs_navline_planed = False
        self.jammer_navline_planed = False

        self.Form_CenterID  = 0
        self.Form_CenterPos = []
        self.Form_Speed     = 200
        self.Form_Heading   = 270
        self.Form_CurrNavPos = [] #当前的导航点

        self.InnerFormation_data = [[(i*(-15)), 35000] for i in range(0,20)]
        self.ActiveDefense_FormData = []
        self.ActiveDefense_EntityList = []
        self.NoMountFighterList=[]
        self.Nomount_fighterFormation=[]
        self.df_rockets = []

        #self.BlueDestroyerWrecked = False
        self.BlueDestroyerInfo = []
        self.BlueDestroyerInfoList = []

        self.BlueNorthCommandInfo = []
        self.BlueSouthCommandInfo = []
        self.ObtaincommandInfo = False

        self.AttackPriorityList = []
        self.AttackPriorityDcided = False
        self.JammerDynCoverScope = 40000

        self.A2C_ObjectFlag = 0  # 1-北指 2-南指 3-驱逐舰
        self.A2NGC_Pos = []
        self.A2NGC_Angle = 270
        self.A2GCP_BomerList = []
        self.A2GCP_BomerA2GAngle = []
        self.A2GCP_BomerA2GPos  = []
        self.A2GCP_EncoyFighter = []
        self.A2GCP_BomerMnulist = []
        self.A2GCP_BomerAttackcmdConfirm = []

        self.A2Command_AttackList = []
        self.OcupyA2GCPPosManuCmdExed = False
        self.EcovyFighterManucmdExed = False
        self.AttackCommandCmdExcuted = False

        # add by lq
        self.blue_air_ids_current = []
        self.air_defense_assign_info_history = pd.DataFrame([], columns = ['target_id', 'target_type', 'air_defense_id', 'air_defense_type', 'start_time', 'end_time'])
        self.own_air_defense_df = pd.DataFrame([], columns=['air_defense_id', 'air_defense_type', 'idle_channel_num', 'current_mun_num'])
        self.step_count = 0
        self.fire_index = 0

        self.stastical_funruntime = []

        self.own_air_defense_dict = dict()
        self.air_defense_assign_history_dict = dict()

        self.max_step_runtime = [0,0]

        self.JammerEvadeState = False

    def reset(self):
        self._init()

    def step(self, sim_time,obs_red,**kwargs):
        curr_time = sim_time
        cmd_list = []

        '''---------------------------------------- 护卫舰初始化-----------------------------------------------'''
        if self.agent_state == 0:
            index = 1
            for ship in obs_red['units']:
                if ship['LX'] == 21:
                    if index == 1:
                        cmd_list.extend(self._ship_movedeploy(ship['ID'], SHIP_POINT1))
                        print('1号护卫舰就位')
                        index += 1
                        continue
                    if index == 2:
                        cmd_list.extend(self._ship_movedeploy(ship['ID'], SHIP_POINT2))
                        print('2号护卫舰就位')
                        index += 1
                        continue
                    if index == 3:
                        cmd_list.extend(self._ship_movedeploy(ship['ID'], SHIP_POINT3))
                        print('3号护卫舰就位')
                        index += 1
                        continue
            self.agent_state = 1

        '''---------------------------------------- 无人机的航线-----------------------------------------------'''
        if self.agent_state == 1:
            index = 1
            for uav in obs_red['units']:
                if uav['LX'] == UnitType.UAV:
                    if index == 1:
                        cmd_list.extend(self._aircraft_areapatrol(uav['ID'], UAV_POINT1, UAV_PATROL_PARAMS_0))
                        index += 1
                        continue
                    if index == 2:
                        cmd_list.extend(self._aircraft_areapatrol(uav['ID'], UAV_POINT2, UAV_PATROL_PARAMS_0))
                        index += 1
                        continue
                    if index == 3:
                        cmd_list.extend(self._aircraft_areapatrol(uav['ID'], UAV_POINT4, UAV_PATROL_PARAMS_0))
                        index += 1
                        continue
            self.agent_state =2

        '''---------------------------------- 空中大编队起飞-----------------------------------------------'''
        if self.agent_state == 2:
            # 空战起飞
            #cmd_list.extend(self._takeoff_areapatrol(1, UnitType.DISTURB, Awacs_Mass_Point, Fighter_MassArea_Params))
            print('干扰机起飞')
            for i in range(4):
                cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2A, Fighter_Mass_Point1, Fighter_MassArea_Params))
            print('空战歼击机起飞')
            for awas in obs_red['units']:
                if awas['LX'] == UnitType.AWACS:
                    cmd_list.extend(self._awacs_patrol(awas['ID'], Awacs_Mass_Point, Awacs_MassArea_Params))
                    self.agent_state = 3
        if self.agent_state ==3:
            for i in range(4):
                cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2G, Bomer_Mass_Point, Bomer_Patrol_Params))
            print('Bomer 1 Fomation Tack off')
            self.agent_state = 4
        if self.agent_state ==4:
            #for i in range(4):
            for i in range(2):
                cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2G, Bomer_Mass_Point, Bomer_Patrol_Params))
            print('Bomer 2 Fomation Tack off')
            self.agent_state = 5
        if self.agent_state ==5:
            # for i in range(8):
            #     cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2G, Bomer_Mass_Point, Bomer_Patrol_Params))
            # print('Bomer 3 Fomation Tack off')
            self.agent_state = 6
        if self.agent_state == 6:
            # 正式发布版本会对模型进行聚合, 即只提供一架干扰机(所以这里作了修改)

            self.agent_state = 7
        if self.agent_state == 7:
            #for i in range(4):
            for i in range(2):
                cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2A, Fighter_Mass_Point2, Fighter_MassArea_Params))
            print('空战歼击机起飞')
            self.agent_state = 8
        if self.agent_state == 8:
            # for i in range(4):
            #     cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2A, Fighter_Mass_Point1, Fighter_MassArea_Params))
            #     pass
            # print('空战歼击机起飞')
            self.agent_state = 9
        if self.agent_state == 9:
            # for i in range(8):
            #     cmd_list.extend(self._takeoff_areapatrol(1, UnitType.A2A, Fighter_Mass_Point2, Fighter_MassArea_Params))
            #     pass
            # print('空战歼击机起飞')
            self.agent_state = 10

        start = time.time()

        dict_units = Dict_GetOwnUnitsData(obs_red['units'])
        dict_qb    = Dict_GetQBData(obs_red['qb'])

        #舰艇防空初始化
        if self.agent_state == 10:
            self.air_defense_init(dict_units)
            self.agent_state = 12

        #控制舰艇防空
        self.gen_blue_airidlist(dict_qb)
        self.air_defense_update(dict_units)
        cmd_list.extend(self.air_defense_planning4(dict_units,dict_qb,sim_time))

        # 获取指挥所信息
        self.GetCommandInfo(dict_qb)
        self.GetBlueDestroyerInfo(dict_qb,sim_time)
        #判断攻击优先级
        self.DecidePriorityAttackList(dict_qb,sim_time)

        #规划航线
        navcmd_list = self.planning_navline(dict_units,sim_time)
        cmd_list.extend(navcmd_list)
        cmd_list.extend(self.JammerEvade(dict_units,dict_qb,sim_time))
        #print("原有机动指令列表长度=",len(navcmd_list))

        #战斗机保持防御队形
        cmd_list.extend(self.Fighter_Keep_DefenseFormation(dict_units,dict_qb,sim_time))

        #空战拦截火力控制
        A2Acmd_list = self.airfight_Firectrl(dict_units, dict_qb, obs_red, sim_time)
        cmd_list.extend(A2Acmd_list)
        self.ManuCmdA2ACmdOptimal(navcmd_list,A2Acmd_list)
        #print("优化后机动指令列表长度=", len(navcmd_list))

        #新版本 攻击舰艇和指挥所
        if len(self.AttackPriorityList)>0:
            a2s_tar = self.AttackPriorityList[0]
            a2s_completed = a2s_tar[2]
            if not a2s_completed:
                #攻击地面目标
                targetflag = a2s_tar[0]
                a2sCmdlist = self.Attack_BlueSurfaceTarget(obs_red, dict_units, dict_qb, targetflag, curr_time)
                cmd_list.extend(a2sCmdlist)
                self.JudgeBlueSurfaceTargetAttackEnd(dict_units, curr_time, targetflag)
            else:
                self.AttackPriorityList.pop(0)

        self.ComputeJammerCoverScope(dict_units,curr_time)

        #机场控制(1小时后有轰炸机就起飞)
        cmd_list.extend(self.AirportControl(obs_red,dict_units, sim_time))

        end = time.time()
        print('=============红方Agent单步长Running time: %s Seconds=====================' % (end - start))

        single_step_runtime = end -start

        if single_step_runtime > self.max_step_runtime[1]:
            self.max_step_runtime = [sim_time, single_step_runtime]
            print("single_step_runtime=",single_step_runtime)
            print("self.last_step_runtime=",self.max_step_runtime[1])

        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!截止目前红方Agent最大步长运行时间=", self.max_step_runtime)

        return cmd_list

    def ComputeJammerCoverScope(self,dict_units,cur_time):
        if cur_time > 240:
            jammerencoyscope = 40000
            #获取干扰机
            jammer_flag, jammer_ID = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.DISTURB)
            if jammer_flag:
                _, jammer_pos = Dict_GetUnitPosByID(dict_units, jammer_ID[0], 3)
                a2s_tar = self.AttackPriorityList[0]
                cur_tarpos = a2s_tar[1][2]
                print("当前攻击的目标位置=",cur_tarpos)
                xdist = jammer_pos[0] - cur_tarpos[0]
                if xdist<=220000 and xdist>=90000:
                    jammerencoyscope = 20000 + (40-20)/(220-90)*(xdist - 90000)
            self.JammerDynCoverScope = jammerencoyscope
            print("cur_time=", cur_time, "干扰掩护范围=", self.JammerDynCoverScope)

    def JammerEvade(self, dict_units, dict_qb, cur_time):
        evade_cmdlist = []

        if cur_time > 240:
            i_flag, blue_FighterTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.A2A, 0)
            a2a_threaten_list = []
            if i_flag:
                _, pos_list = Dict_GetUnitsPosByIDList(dict_qb, blue_FighterTarList, 2)
                distance_list = [calculate_2d_distance(self.Form_CenterPos, b_pos) for b_pos in pos_list]
                index = 0
                for val in distance_list:
                    if val <= 130000:
                        a2a_threaten_list.append((blue_FighterTarList[index], val))
                    index += 1

            jammer_flag, jammer_ID = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.DISTURB)
            if jammer_flag:
                _, jammer_pos = Dict_GetUnitPosByID(dict_units, jammer_ID[0], 3)
                _, jammer_heading = Dict_GetUnitHeadingByID(dict_units, jammer_ID[0])
                _, jammer_speed = Dict_GetUnitSpeedByID(dict_units, jammer_ID[0])

                threaten_vector = []
                if len(a2a_threaten_list)>2:
                    threaID_list = [index[0] for index in a2a_threaten_list]
                    dist_list = [idx[1] for idx in a2a_threaten_list]
                    flag, pos_list = Dict_GetUnitsPosByIDList(dict_qb, threaID_list, 2)
                    for i in range(len(pos_list)):
                        threaten_vector.append([jammer_pos[0]-pos_list[i][0],jammer_pos[1]-pos_list[i][1]])
                    print("threaten_vector=",threaten_vector)

                    evade_array = np.array([0,0])
                    threaten_array = np.array(threaten_vector)
                    for j in range(len(threaten_vector)):
                        evade_array = evade_array + (130000 - dist_list[j])*threaten_array[j]/lengthofVector(threaten_vector[j])
                    print("evade_array=",evade_array)
                    evade_params = [
                        Point(jammer_pos[0]+1000*evade_array[0]/lengthofVector(evade_array), jammer_pos[1]+1000*evade_array[1]/lengthofVector(evade_array), jammer_pos[2]),
                        Point(jammer_pos[0]+2000*evade_array[0], jammer_pos[1]+2000*evade_array[1], jammer_pos[2])]
                    evadetarpos = [jammer_pos[0] + 500 * evade_array[0], jammer_pos[1] + 500 * evade_array[1], jammer_pos[2]]
                    print("evadetarpos=",evadetarpos)
                    evade_cmdlist.extend(self._aircraft_linepatrol(jammer_ID[0], 220, evade_params))
                    self.JammerEvadeState = True
                else:
                    self.JammerEvadeState = False
            else:
                self.JammerEvadeState = False
        return evade_cmdlist

    #控制进攻大编队队形
    def planning_navline(self,dict_units,cur_time):
        navline_cmdlist=[]
        self.Form_CurrNavPos = NORTH_NAVLINE_POS

        #获取干扰机机动状态数据
        jammer_flag, jammer_ID = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.DISTURB)
        if jammer_flag:
            _, jammer_pos = Dict_GetUnitPosByID(dict_units, jammer_ID[0], 3)
            _, jammer_heading = Dict_GetUnitHeadingByID(dict_units, jammer_ID[0])
            _, jammer_speed = Dict_GetUnitSpeedByID(dict_units, jammer_ID[0])
            self.Form_CenterID  = jammer_ID[0]
            self.Form_CenterPos = jammer_pos
            self.Form_Speed = jammer_speed
            self.Form_Heading = jammer_heading
            if cur_time<240:
                Jline_params = [Point(jammer_pos[0], jammer_pos[1], jammer_pos[2]),
                                Point(150000,10000,jammer_pos[2])]
                navline_cmdlist.extend(self._disturb_linepatrol(jammer_ID[0], Jline_params))
            # 规划预警机航线
            if cur_time > 240:
                if self.jammer_navline_planed == False:
                    navline_cmdlist.extend(self._awacs_patrol(jammer_ID[0],
                                    [NORTH_NAVLINE_POS[0], NORTH_NAVLINE_POS[1], 7500], AWACS_PATROL_PARAMS))
                    self.opration_stage = RedAgentOperationStage.Formation_Navaigation
                    self.jammer_navline_planed = True

        # 规划预警机在编队的位置控制
        awacs_flag, awacs_ID = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.AWACS)
        if awacs_flag:
            _, awacs_pos = Dict_GetUnitPosByID(dict_units, awacs_ID[0], 3)
            _, awacs_heading = Dict_GetUnitHeadingByID(dict_units, awacs_ID[0])
            _, awacs_speed = Dict_GetUnitSpeedByID(dict_units, awacs_ID[0])
            if cur_time > 240:
                if jammer_flag == True:
                    Awacs_Fomrabspos, Awacs_Formabsdirection =self.compute_formation_abspos(self.Form_CenterPos,Awacs_FormationPos,self.Form_Heading)
                    #计算出最优速度和机动航线
                    formationspd_vector = CalcEdPosByDisAngle(self.Form_Speed,self.Form_Heading)
                    dist_vector = [(Awacs_Fomrabspos[0][0]-awacs_pos[0]),(Awacs_Fomrabspos[0][1]-awacs_pos[1])]
                    opt_spdvector = optimalSpeed(list(formationspd_vector), 220, dist_vector)
                    print("预警机最优速度=",opt_spdvector)
                    opt_speed = lengthofVector(opt_spdvector)
                    if (opt_speed <= 220):
                        opt_speed = 200
                    Awacs_params = [Point(awacs_pos[0]+10*opt_spdvector[0], awacs_pos[1]+10*opt_spdvector[1], awacs_pos[2]),
                                   Point(Awacs_Fomrabspos[0][0]+30*opt_spdvector[0],Awacs_Fomrabspos[0][1]+30*opt_spdvector[1],awacs_pos[2])]
                    navline_cmdlist.extend(self._aircraft_linepatrol(awacs_ID[0], opt_speed, Awacs_params))
                else:
                    if self.awacs_navline_planed == False:
                        navline_cmdlist.extend(self._aircraft_areapatrol(awacs_ID[0], [self.Form_CurrNavPos[0], self.Form_CurrNavPos[1], 7500],AWACS_PATROL_PARAMS))
                        self.opration_stage = RedAgentOperationStage.Formation_Navaigation
                        self.awacs_navline_planed = True

        # 获取预警机机动状态数据
        # awacs_flag, awacs_ID = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.AWACS)
        # if awacs_flag:
        #     _, awacs_pos = Dict_GetUnitPosByID(dict_units, awacs_ID[0], 3)
        #     _, awacs_heading = Dict_GetUnitHeadingByID(dict_units, awacs_ID[0])
        #     _, awacs_speed = Dict_GetUnitSpeedByID(dict_units, awacs_ID[0])
        #     self.Form_CenterID  = awacs_ID[0]
        #     self.Form_CenterPos = awacs_pos
        #     self.Form_Speed = awacs_speed
        #     self.Form_Heading = awacs_heading
        #     # 规划预警机航线
        #     if cur_time > 240:
        #         if self.awacs_navline_planed == False:
        #             navline_cmdlist.extend(self._awacs_patrol(awacs_ID[0],
        #                             [NORTH_NAVLINE_POS[0], NORTH_NAVLINE_POS[1], 7500], AWACS_PATROL_PARAMS))
        #             self.Form_CurrNavPos = NORTH_NAVLINE_POS
        #             self.opration_stage = RedAgentOperationStage.Formation_Navaigation
        #             self.awacs_navline_planed = True

        #获取干扰机机动状态数据
        # jammer_flag, jammer_ID = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.DISTURB)
        # if jammer_flag:
        #     _, jammer_pos = Dict_GetUnitPosByID(dict_units, jammer_ID[0], 3)
        #     _, jammer_heading = Dict_GetUnitHeadingByID(dict_units, jammer_ID[0])
        #     _, jammer_speed = Dict_GetUnitSpeedByID(dict_units, jammer_ID[0])
        #     if cur_time<240:
        #         Jline_params = [Point(jammer_pos[0], jammer_pos[1], jammer_pos[2]),
        #                         Point(150000,10000,jammer_pos[2])]
        #         navline_cmdlist.extend(self._disturb_linepatrol(jammer_ID[0], Jline_params))
        #     # 规划干扰机航线
        #     if cur_time > 240:
        #         if awacs_flag == True:
        #             Jammer_Fomrabspos, Jammer_Formabsdirection =self.compute_formation_abspos(self.Form_CenterPos,Jammer_FormationPos,self.Form_Heading)
        #             #计算出最有速度和机动航线
        #             formationspd_vector = CalcEdPosByDisAngle(self.Form_Speed,self.Form_Heading)
        #             dist_vector = [(Jammer_Fomrabspos[0][0]-jammer_pos[0]),(Jammer_Fomrabspos[0][1]-jammer_pos[1])]
        #             opt_spdvector = optimalSpeed(list(formationspd_vector), 220, dist_vector)
        #             print("干扰机最有速度=",opt_spdvector)
        #             opt_speed = lengthofVector(opt_spdvector)
        #             if (opt_speed <= 220):
        #                 opt_speed = 200
        #             Jline_params = [Point(jammer_pos[0]+10*opt_spdvector[0], jammer_pos[1]+10*opt_spdvector[1], jammer_pos[2]),
        #                            Point(Jammer_Fomrabspos[0][0]+30*opt_spdvector[0],Jammer_Fomrabspos[0][1]+30*opt_spdvector[1],jammer_pos[2])]
        #             navline_cmdlist.extend(self._aircraft_linepatrol(jammer_ID[0], opt_speed, Jline_params))
        #         else:
        #             navline_cmdlist.extend(self._aircraft_areapatrol(jammer_ID[0], [self.Form_CurrNavPos[0], self.Form_CurrNavPos[1], 7500],AWACS_PATROL_PARAMS))

        #规划轰炸机队形
        bomer_flag, bomerID_List = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.A2G)
        bomerFormationIDList = bomerID_List
        if bomer_flag:
            # 弹药为0的飞机不受编队队形控制
            returnbaselist=[]
            for bomID in bomerID_List:
                _ , bomerMount = Dict_GetUnitMunNumByID(dict_units,bomID)
                if bomerMount==0:
                    returnbaselist.append(bomID)
                    #print("轰炸机=", bomID, "返航")
                    navline_cmdlist.extend(self._returntobase(bomID))
            bomerFormationIDList = list(set(bomerFormationIDList)-set(returnbaselist))
            #新版本
            bomerFormationIDList = list(set(bomerFormationIDList) - set(self.A2GCP_BomerList))
            #如果预警机和电子干扰击都被干掉了,
            iflag,center_pos = Dict_GetUnitPosByID(dict_units,self.Form_CenterID,2)
            if iflag:
                CenterIDtype = dict_units[self.Form_CenterID].Type
                if CenterIDtype == UnitType.A2G:
                    # self.Form_CenterID is center point of a2g formation
                    bomerFormationIDList = list(set(bomerFormationIDList)-set([self.Form_CenterID]))
            _, bomer_poslist = Dict_GetUnitsPosByIDList(dict_units, bomerFormationIDList, 3)
            Bomer_Fomrabspos, Bomer_Formabsdirection = self.compute_formation_abspos(self.Form_CenterPos,Bomer_FormationPos,self.Form_Heading)

        # 规划Bomer航线
        if cur_time > 240:
            for i in range(len(bomerFormationIDList)):
                bomerId = bomerFormationIDList[i]
                flag, bomerpos = Dict_GetUnitPosByID(dict_units,bomerId,3)

                formationspd_vector = CalcEdPosByDisAngle(self.Form_Speed,self.Form_Heading)
                dist_vector = [(Bomer_Fomrabspos[i][0]-bomerpos[0]),(Bomer_Fomrabspos[i][1]-bomerpos[1])]
                opt_spdvector = optimalSpeed(list(formationspd_vector), 220, dist_vector)
                #print("轰炸机最优速度=",opt_spdvector)
                opt_speed = lengthofVector(opt_spdvector)
                if (opt_speed <= 220):
                    opt_speed = 200

                Bomer_lineparams = [Point(bomerpos[0]+10*opt_spdvector[0], bomerpos[1]+10*opt_spdvector[1], bomerpos[2]),
                           Point(Bomer_Fomrabspos[i][0]+30*opt_spdvector[0],Bomer_Fomrabspos[i][1]+30*opt_spdvector[1],bomerpos[2])]
                navline_cmdlist.extend(self._aircraft_linepatrol(bomerId, opt_speed, Bomer_lineparams))

        #遇到意外情况时灵活调整大编队中心 0-预警机 1-干扰机 2-轰炸机编队
        if  jammer_flag== False:
            if awacs_flag:
                self.Form_CenterID = awacs_ID[0]
                self.Form_CenterPos = awacs_pos
                self.Form_Heading  = awacs_heading
                self.Form_Speed    = awacs_speed
            else:
                #预警机和干扰机都被击落,选择距离中心点最近的轰炸机作为中心点
                BomertoCenterList = []
                bflag, bomerIDList = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.A2G)
                if bflag:
                    bflag, bomerPoslist = Dict_GetUnitsPosByIDList(dict_units, bomerIDList, 2)
                    for i in range(len(bomerPoslist)):
                        flag, toCenterdist = CalcP2PDistance(bomerPoslist[i], self.Form_CenterPos, 2)
                        BomertoCenterList.append([bomerIDList[i], toCenterdist])
                    BomertoCenterList = sorted(BomertoCenterList, key=lambda kv: (kv[1], kv[0]))
                    newCenterID = BomertoCenterList[0][0]
                    iflag, newCenterpos = Dict_GetUnitPosByID(dict_units,newCenterID,2)
                    if iflag:
                        _, newCenterSpd = Dict_GetUnitSpeedByID(dict_units,newCenterID)
                        _, newCenterHeading = Dict_GetUnitHeadingByID(dict_units,newCenterID)
                        self.Form_CenterID  = newCenterID
                        self.Form_CenterPos = newCenterpos
                        self.Form_Speed     = newCenterSpd
                        self.Form_Heading   = newCenterHeading
                        #print("重新制定的编队中心点的速度和航向=",[self.Form_Speed,self.Form_Heading])
                        navline_cmdlist.extend(self._aircraft_areapatrol(self.Form_CenterID,
                                                                  [self.Form_CurrNavPos[0], self.Form_CurrNavPos[1], 7500],
                                                                  AWACS_PATROL_PARAMS))
                        print("重新制定的编队中心航线点=", [self.Form_CurrNavPos[0], self.Form_CurrNavPos[1]])

        # 规划护卫舰的进攻支援航线
        navline_cmdlist.extend(self.Ship_Navline_Planning(dict_units,cur_time))

        return navline_cmdlist

    def Fighter_Keep_DefenseFormation(self,dict_units,dict_qb,cur_time):
        # 规划护航战斗机编队航线,保持防御阵型
        navline_cmdlist = []
        if cur_time>360:
            fighter_manuver_list = self.keep_Defense_formation(dict_units, dict_qb)
            navline_cmdlist.extend(fighter_manuver_list)
        return navline_cmdlist

    #规划护卫舰的航线,支援掩护轰炸机进攻作战行动
    def Ship_Navline_Planning(self, dict_units, cur_time):
        nav_cmdlist = []
        iflag, shipIDlist = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.SHIP)
        if iflag and cur_time>120:
            _, shipPos_list = Dict_GetUnitsPosByIDList(dict_units, shipIDlist, 2)
            for i in range(len(shipPos_list)):
                if shipPos_list[i][1]>0:
                    #支援北岛进攻的护卫舰
                    if len(self.BlueNorthCommandInfo)>0:
                        if (self.BlueNorthCommandInfo[1] == 1):
                            NorComandtoShipAz = CalcAzByTwoPts(self.BlueNorthCommandInfo[2],shipPos_list[i])
                            NorthShipStandbypos = CalcDesPtByAzRadius(self.BlueNorthCommandInfo[2], NorComandtoShipAz, 127000)
                            nav_cmdlist.extend(self._ship_areapatrol(shipIDlist[i], [NorthShipStandbypos[0],NorthShipStandbypos[1],0]))

                    # #支援舰艇的进攻作战
                    # if len(self.BlueDestroyerInfo)>0:
                    #     BshiptoRShipAz = CalcAzByTwoPts(self.BlueDestroyerInfo[2], shipPos_list[i])
                    #     NorthShipStandbypos = CalcDesPtByAzRadius(self.BlueDestroyerInfo[2], BshiptoRShipAz, 127000)
                    #     nav_cmdlist.extend(self._ship_areapatrol(shipIDlist[i], [NorthShipStandbypos[0],NorthShipStandbypos[1],0]))
                else:
                    pass
        return nav_cmdlist

    #保持空中战斗机编队的防御阵型
    def keep_Defense_formation(self,dict_units,dict_qb):
        manuver_list = []

        #防御圈内部可空战的战斗机列表
        InnerDef_A2AFigterList=[]

        # 找出离各位置点最近的护航飞机,并分别下达航线巡逻指令
        i_flag,A2A_FighterID_list = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.A2A)

        self.airfight_target_assign(dict_units, dict_qb, [0, 250000])

        InnerDef_FigterList = list(set(A2A_FighterID_list)-set(self.ActiveDefense_EntityList))

        self.NoMountFighterList = []
        self.Nomount_fighterFormation = []
        # 统计参与内部防御的战斗机余弹量, 并按弹量从大到小排序
        flag, f_milnum_list = Dict_GetUnitsMunNumByIDList(dict_units, InnerDef_FigterList)
        combine_list = []
        if flag:
            for j in range(len(InnerDef_FigterList)):
                combine_list.append((InnerDef_FigterList[j],f_milnum_list[j]))
                if f_milnum_list[j]==0:
                    self.NoMountFighterList.append(InnerDef_FigterList[j])
                combine_list = sorted(combine_list, key=lambda x: (x[1], x[0]), reverse=True)
            InnerDef_FigterList= [val[0] for val in combine_list]
            InnerDef_A2AFigterList = list(set(InnerDef_FigterList)-set(self.NoMountFighterList))
            #print("内部防御圈实体列表=",InnerDef_A2AFigterList)

        if len(self.ActiveDefense_EntityList) > 0:
            #主动防御圈队形保持计算
            self.ActiveDefense_FormData = sorted(self.ActiveDefense_FormData,key=lambda x:(x[0],x[1]))
            #print("主动防御圈实体列表=", self.ActiveDefense_EntityList)
            InnerDef_refaz = ((self.ActiveDefense_FormData[0][0] + self.ActiveDefense_FormData[-1][0]))/2
            #print("内部防御圈中心参考角=", InnerDef_refaz)

            #生成内部防御圈队形数据
            Inner_formdata=[]
            for k in range(len(InnerDef_A2AFigterList)):
                if (k%2 == 0):
                    reaz = InnerDef_refaz + -(k/2)*15
                else:
                    reaz = InnerDef_refaz + (k+1)*15/2
                Inner_formdata.append([reaz,self.JammerDynCoverScope-5000])
            if len(Inner_formdata)>0:
                Inner_formdata = sorted(Inner_formdata, key=lambda x: (x[0], x[1]), reverse=True)
            #print("内防御圈的队形数据=",Inner_formdata)
            self.InnerFormation_data = Inner_formdata

            #无弹飞机的计算
            for j in range(len(self.NoMountFighterList)):
                if (j%2)==0:
                    nmreaz = InnerDef_refaz + -(j/2)*15
                else:
                    nmreaz = InnerDef_refaz + (j+1)*15/2
                self.Nomount_fighterFormation.append([nmreaz,self.JammerDynCoverScope-8000])
            if (len(self.Nomount_fighterFormation))>0:
                self.Nomount_fighterFormation = sorted(self.Nomount_fighterFormation, key=lambda x: (x[0], x[1]), reverse=True)

        #else:
            #self.InnerFormation_data = [[(i*(-15)), 35000] for i in range(0,16)]
        if len(self.ActiveDefense_EntityList)==0:
            self.InnerFormation_data = [[(i * (-15)), self.JammerDynCoverScope-2000] for i in range(0, 20)]
            InnerDef_A2AFigterList = InnerDef_A2AFigterList + self.NoMountFighterList
            self.NoMountFighterList=[]

        #内层防御实体保持编队的航线机动命令
        print("内部防御战斗机列表=",InnerDef_A2AFigterList)
        InnerFighter_ManuCmdList = self.GennavlinesCmd(dict_units,InnerDef_A2AFigterList,self.InnerFormation_data)
        manuver_list.extend(InnerFighter_ManuCmdList)

        #外层防御实体保持编队的机动命令
        print("外部防御战斗机列表=", self.ActiveDefense_EntityList)
        DealThreaFighter_ManuCmdList = self.GennavlinesCmd(dict_units, self.ActiveDefense_EntityList,
                                                           self.ActiveDefense_FormData)
        manuver_list.extend(DealThreaFighter_ManuCmdList)

        #无弹飞机保持编队
        NoMountFighter_ManuCmdList = self.GennavlinesCmd(dict_units,self.NoMountFighterList,self.Nomount_fighterFormation)
        manuver_list.extend(NoMountFighter_ManuCmdList)

        return manuver_list

    #GennavlinesCmd
    def GennavlinesCmd(self,dict_units,A2A_List,Form_data):

        manucmd_list = []

        if len(self.A2GCP_EncoyFighter) > 0:
            A2A_List = list(set(A2A_List) - set(self.A2GCP_EncoyFighter))

        if len(A2A_List)>0:

            _,A2AFighter_Poslist = Dict_GetUnitsPosByIDList(dict_units, A2A_List, 2)

            if len(A2A_List)<len(Form_data):
                Form_data = [Form_data[i] for i in range(len(A2A_List))]

            if self.JammerEvadeState == False:
                scort_abspos, escort_absdirection = self.compute_formation_abspos(self.Form_CenterPos, Form_data, self.Form_Heading)
            else:
                self.Form_CenterPos = [self.Form_CenterPos[0]-30000,self.Form_CenterPos[1]]
                scort_abspos, escort_absdirection = self.compute_formation_abspos(self.Form_CenterPos, Form_data,
                                                                                  self.Form_Heading)

            formation_posassign_list = minMaxWeightMatchingCppVer(scort_abspos, A2AFighter_Poslist)
            print("战斗机护航编队防御位置分配表=",formation_posassign_list)

            for Serve_Tuple in formation_posassign_list:
                if Serve_Tuple[1] == -1:
                    continue
                fighter_id = A2A_List[Serve_Tuple[1]]
                i_flag,fighter_pos = Dict_GetUnitPosByID(dict_units,fighter_id,3)
                if i_flag:
                    fcur_pos = fighter_pos
                    # #计算出最优化速度矢量
                    #print("self.Form_Speed=",self.Form_Speed,"self.Form_Heading=",self.Form_Heading)
                    formationspd_vector = CalcEdPosByDisAngle(self.Form_Speed,self.Form_Heading)
                    dist_vector = [(scort_abspos[Serve_Tuple[0]][0]-fcur_pos[0]),\
                                  (scort_abspos[Serve_Tuple[0]][1] - fcur_pos[1])]
                    opt_spdvector = optimalSpeed(list(formationspd_vector), Fighter_Max_Speed, dist_vector)
                    opt_speed = lengthofVector(opt_spdvector)
                    if (opt_speed <= Fighter_min_Speed):
                        opt_speed = Fighter_min_Speed
                    #print("当前实体=",fighter_id,"最优速度=",opt_speed)
                    line_params = [Point(fcur_pos[0]+opt_spdvector[0]*10,fcur_pos[1]+opt_spdvector[1]*10,fcur_pos[2]),
                                   Point(scort_abspos[Serve_Tuple[0]][0],scort_abspos[Serve_Tuple[0]][1], fcur_pos[2])]
                    #判断当前飞机是否被导弹攻击
                    #flag,attackerlist = JudgeAircraftBeAttacked_ByID(self.df_rockets,fighter_id)
                    #if flag == False:
                    manucmd_list.extend(self._aircraft_linepatrol(fighter_id,opt_speed,line_params))
        #print("战斗机护航编队机动航线指令=",len(manucmd_list))
        return manucmd_list

    #预估导弹飞行时间
    def predict_milssleflytime(self,obs_red,targetID,attackID,dim=2):
        distance = 0.0
        TargetPos = []
        milflytime = 0.0
        milSpeed   = 300

        for blue_unit in obs_red['qb']:
            if blue_unit['ID'] == targetID:
                TargetPos.extend([blue_unit['X'],blue_unit['Y'],blue_unit['Z']])
                if blue_unit['LX'] == 21 or blue_unit['LX'] == 41:
                    milSpeed = 777.0
                else:
                    milSpeed = 1000.0
                break
        for red_unit in obs_red['units']:
            if red_unit['ID'] == attackID and len(TargetPos)>0:
                if dim == 2:
                    distance = math.sqrt(
                        math.pow((red_unit['X']-TargetPos[0]), 2) + math.pow((red_unit['Y']-TargetPos[1]), 2))
                    #print("distance=",distance)
                if dim == 3:
                    distance = math.sqrt(
                        math.pow(red_unit['X'] - TargetPos[0], 2) + math.pow(red_unit['Y'] - TargetPos[1], 2) +
                        math.pow(red_unit['Z'] - TargetPos[2], 2))
                break
        if (distance>0):
            milflytime = (distance)/milSpeed
            #print("导弹预估飞行时间=",milflytime)
        return milflytime

    #更新空战拦截列表
    def update_a2aIntercepList(self,cur_simtime):
        del_count = 0
        for i in range(len(self.a2a_intercept_list)):
            #当前仿真时间与导弹发射时间的差与导弹预估飞行时间
            recept_record = self.a2a_intercept_list[i-del_count]
            if (cur_simtime - recept_record[0]) > recept_record[3]:
                del self.a2a_intercept_list[i-del_count]
                del_count += 1

    #主动空战防御目标最近匹配
    def airfight_target_assign(self,dict_units,dict_qb,dist_limits,Theta=0.5):
        #统计预警机0km-115公里范围内的敌方空中战斗机目标
        i_flag, blue_FighterTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.A2A,0)
        a2a_threaten_tarlist = []
        if i_flag:
            _, pos_list = Dict_GetUnitsPosByIDList(dict_qb, blue_FighterTarList, 2)
            distance_list = [calculate_2d_distance(self.Form_CenterPos,b_pos) for b_pos in pos_list]
            index = 0
            for val in distance_list:
                if val<=dist_limits[1] and val>=dist_limits[0]:
                    a2a_threaten_tarlist.append((blue_FighterTarList[index],val))
                index+=1
        self.ActiveDefense_FormData = []
        self.ActiveDefense_EntityList = []
        if len(a2a_threaten_tarlist)>0:
            #print("威胁目标表=",a2a_threaten_tarlist)
            # 对敌方空中目标进行聚类分析
            #distfilter_list = [val[1] for val in a2a_threaten_tarlist]
            #classes, centerIndex  = maxmin_distance_cluster(distfilter_list, Theta)
            #print("classes=",classes,"centerIndex:=",centerIndex)

            #--------------------寻找目标最近的护航战斗机进行拦截-------------------------#
            #找出余弹量大于1的我方战斗机(能进行空战拦截)
            _,a2a_redIDList = Dict_GetOwnUnitsID_ByType(dict_units,UnitType.A2A)
            i_flag, f_milnum_list = Dict_GetUnitsMunNumByIDList(dict_units, a2a_redIDList)
            A2A_CanAssigned_List = []
            for i in range(len(f_milnum_list)):
                if f_milnum_list[i] > 0:
                    A2A_CanAssigned_List.append(a2a_redIDList[i])
                    # print("A2A_CanAssigned_List=",A2A_CanAssigned_List)

            #寻找能最快接近敌方威胁战斗机的我方战斗机(1V1匹配)
            Match_list = []
            threaten_TargetIDList = [val[0] for val in a2a_threaten_tarlist]
            _,TarPos_list = Dict_GetUnitsPosByIDList(dict_qb,threaten_TargetIDList,2)
            _,assignPos_list = Dict_GetUnitsPosByIDList(dict_units,A2A_CanAssigned_List,2)
            assign_list = minMaxWeightMatchingCppVer(TarPos_list,assignPos_list)
            #assign_list = sorted(assign_list, key=lambda x: (x[0], x[1]))
            for assign_val in assign_list:
                if assign_val[1]==-1:
                    continue
                tmpRedID = A2A_CanAssigned_List[assign_val[1]]
                tmpBlueID = threaten_TargetIDList[assign_val[0]]
                Match_list.append((tmpBlueID,tmpRedID))
                #生成空战主动防御圈的队形数据
                ipos_flag,threaten_Tarpos = Dict_GetUnitPosByID(dict_qb,tmpBlueID,2)
                if ipos_flag:
                    rela_az = CalcAzByTwoPts(self.Form_CenterPos,threaten_Tarpos)#rela_az 与y轴夹角  [-180,180]
                    self.ActiveDefense_FormData.append((rela_az, self.JammerDynCoverScope-2000))
                    self.ActiveDefense_EntityList.append(tmpRedID)
                #print("tRedID=",tmpRedID,"防御目标",tmpBlueID)
            #print("主动防御匹配表=",Match_list)

            #


    #空战开火控制
    def airfight_Firectrl(self,dict_units,dict_qb,obs_red,cur_time):
        interceptcmd_list = []

        self.update_a2aIntercepList(cur_time)

        i_flag, blue_FighterTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.A2A,0)
        a2a_thratentarlist = []
        a2a_tarIDList = []
        if i_flag:
            _, pos_list = Dict_GetUnitsPosByIDList(dict_qb, blue_FighterTarList, 2)
            distance_list = [calculate_2d_distance(self.Form_CenterPos, b_pos) for b_pos in pos_list]
            a2a_thratentarlist = [(blue_FighterTarList[i], distance_list[i]) for i in range(len(distance_list)) if
                           distance_list[i]<250000]
            a2a_tarIDList = [ elem[0] for elem in a2a_thratentarlist]
            _, a2a_tarposlist = Dict_GetUnitsPosByIDList(dict_qb, a2a_thratentarlist,2)
        #print("空战威胁表=",a2a_thratentarlist,"空战威胁目标ID表=",a2a_tarIDList)

        #战斗机的攻击控制
        _, a2a_redA2AIDList = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.A2A)
        # a2a_DistList = [[(CalcDistByTwoUnitID(dict_units,dict_qb,a2aID, threatenID,2))[1],(a2aID,threatenID)] for a2aID in a2a_redA2AIDList
        #                                     for threatenID in a2a_tarIDList
        #                 if (CalcDistByTwoUnitID(dict_units,dict_qb, a2aID, threatenID,2))[0]]
        a2a_DistList = []
        if (len(a2a_redA2AIDList)>0 and len(a2a_tarIDList)>0):
            iflag, a2a_DistList = Dict_CalcRelDistByTwoUnitIDList(dict_units, dict_qb, a2a_redA2AIDList, a2a_tarIDList,2)
            a2a_DistList = sorted(a2a_DistList,key=lambda x:(x[0],x[1]))
        #print("a2a_DistList=",a2a_DistList)

        for i in range(len(a2a_DistList)):
            a2adist = a2a_DistList[i][0]
            a2a_tuple = a2a_DistList[i][1]
            redid = a2a_tuple[0]
            blueid = a2a_tuple[1]
            if self.count_targetnum_a2aList(a2a_tuple[1]) < 3 and a2adist<100000:
                print("{}拦截{}".format(redid,blueid))
                interceptcmd_list.extend(self._airattack(redid, blueid))
                preflytime = self.predict_milssleflytime(obs_red, blueid ,redid)
                self.a2a_intercept_list.append([cur_time,blueid , redid, preflytime])

        #预警机和轰炸机的攻击控制
        iflag, blue_ImportantTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.AWACS, 0)
        iflag, blue_BomerTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.A2G, 0)
        blue_ImportantTarList += blue_BomerTarList

        a2importTar_DistList = []
        if (len(a2a_redA2AIDList)>0 and len(blue_ImportantTarList)>0):
            iflag, a2importTar_DistList = Dict_CalcRelDistByTwoUnitIDList(dict_units, dict_qb, a2a_redA2AIDList, blue_ImportantTarList,2)
            a2importTar_DistList = sorted(a2importTar_DistList,key=lambda x:(x[0],x[1]))

        for i in range(len(a2importTar_DistList)):
            a2adist = a2importTar_DistList[i][0]
            a2a_tuple = a2importTar_DistList[i][1]
            redid = a2a_tuple[0]
            blueid = a2a_tuple[1]
            if self.count_targetnum_a2aList(a2a_tuple[1]) < 2 and a2adist<100000:
                interceptcmd_list.extend(self._airattack(redid, blueid))
                preflytime = self.predict_milssleflytime(obs_red, blueid ,redid)
                self.a2a_intercept_list.append([cur_time,blueid , redid, preflytime])

        if len(interceptcmd_list)>0:
            #print("拦截指令列表=",interceptcmd_list)
            pass

        return interceptcmd_list

    def gen_blue_airidlist(self,dict_qb):

        i_flag, blue_FighterTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.A2A, 0)
        iflag, blue_awacsTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.AWACS, 0)
        iflag, blue_BomerTarList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.A2G, 0)

        self.blue_air_ids_current = blue_FighterTarList + blue_awacsTarList + blue_BomerTarList
        print('blue air targets:' % self.blue_air_ids_current)

    #统计空战拦截列表中当前正在被打击的敌战斗机目标数量
    def count_targetnum_a2aList(self,targetID):
        targetID_count = 0
        for i in range(len(self.a2a_intercept_list)):
            recept_record = self.a2a_intercept_list[i]
            if (targetID == recept_record[1]):
                targetID_count += 1
        return  targetID_count

    # 计算出参考飞机位置点(预警机)和队形数据,计算出护航编队各飞机的位置点
    def compute_formation_abspos(self,center_pos,Formation_data,center_heading):
        abs_pos = []
        abs_direction = []
        #print("队形数据=",Formation_data)
        for form_data in Formation_data:
            # if relahead==True:
            #     abs_heading = (form_data[0] + center_heading)%360
            # else:
            abs_heading = form_data[0]#正北为0的夹角(角度)
            abs_angle = 90-abs_heading#然后转换为极坐标下的角度
            abs_direction.append(abs_angle)
            px,py = CalcEdPosByDisAngle(form_data[1],abs_direction[-1])
            abs_pos.append([px+center_pos[0],py+center_pos[1]])
        return abs_pos,abs_direction

    #目标攻击表的优先级
    def DecidePriorityAttackList(self,dict_qb,cur_time):

        shiptoScomdist = 10000000
        shiptoNComdist = 10000000

        ship_YPosList = []
        for shipInfo_idx in self.BlueDestroyerInfoList:
            ship_ypos = shipInfo_idx[2][1]
            ship_id = shipInfo_idx[0]
            ship_YPosList.append([ship_id, ship_ypos])
        ship_YPosList = sorted(ship_YPosList, key=lambda kv: (kv[1], kv[0]), reverse=True)
        tmp_list = []
        for idx in ship_YPosList:
            for shipInfo_idx in self.BlueDestroyerInfoList:
                if shipInfo_idx[0] == idx[0]:
                    tmp_list.append(shipInfo_idx)
        self.BlueDestroyerInfoList = tmp_list
        if len(self.BlueDestroyerInfoList)>0:
            self.BlueDestroyerInfo = self.BlueDestroyerInfoList[0]
            print("cur_time=",cur_time,"self.BlueDestroyerInfo=",self.BlueDestroyerInfo)

        if self.AttackPriorityDcided == False:
            if len(self.BlueDestroyerInfoList)>0 and len(self.AttackPriorityList)>0 and cur_time>130:
                #发现舰艇后，根据条件重新设定攻击优先级
                if self.BlueDestroyerInfo[2][0] > -295000 and self.BlueDestroyerInfo[2][0]<=0:
                    if len(self.BlueSouthCommandInfo)>0:
                        iflag, shiptoScomdist = CalcP2PDistance(self.BlueDestroyerInfo[2],self.BlueSouthCommandInfo[2],2)
                    if len(self.BlueNorthCommandInfo)>0:
                        iflag, shiptoNComdist = CalcP2PDistance(self.BlueDestroyerInfo[2],self.BlueNorthCommandInfo[2],2)
                    if shiptoNComdist < shiptoScomdist:
                        if self.opration_stage == RedAgentOperationStage.Formation_Navaigation or self.opration_stage\
                                 == RedAgentOperationStage.Mass_Force:
                            self.AttackPriorityList.insert(0,['D', self.BlueDestroyerInfo,False])
                        if self.opration_stage == RedAgentOperationStage.Attack_Stage:
                            self.AttackPriorityList.insert(1,['D', self.BlueDestroyerInfo, False])
                    else:
                        if len(self.AttackPriorityList)>=1:
                            self.AttackPriorityList.insert(0,['D', self.BlueDestroyerInfo, False])
                        else:
                            self.AttackPriorityList.insert(1, ['D', self.BlueDestroyerInfo, False])
                    print("cur_time=",cur_time,"self.AttackPriorityList=",self.AttackPriorityList)
                    self.AttackPriorityDcided = True

    # version - 2
    def GetBlueDestroyerInfo(self,dict_qb,cur_time):
        if cur_time>130:
            FindShipIDList = []
            for shipInfo_idx in self.BlueDestroyerInfoList:
                FindShipIDList.append(shipInfo_idx[0])

            bflag, DestroyerIDList  = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.SHIP,0)
            if bflag:
                for shipID_idx in DestroyerIDList:
                    dflag, Destroyer_Pos = Dict_GetUnitPosByID(dict_qb, shipID_idx, 2)
                    if dflag:
                        if shipID_idx not in FindShipIDList:
                            self.BlueDestroyerInfoList.append([shipID_idx, 1, Destroyer_Pos])
                        else:
                            self.BlueDestroyerInfo = [shipID_idx, 1, Destroyer_Pos]
            else:
                if len(FindShipIDList)>0:
                    for i in range(len(self.BlueDestroyerInfoList)):
                        shipInfolist = self.BlueDestroyerInfoList[i]
                        shipidx = shipInfolist[0]
                        dflag, isAlvie = Dict_GetUnitAliveByID(dict_qb, shipidx)
                        print("当前时间=",cur_time,"护卫舰=",shipidx,"存活状态=",isAlvie)
                        if dflag == False:
                            self.BlueDestroyerInfoList[i][1] = 0
                            if self.BlueDestroyerInfo[0] == shipidx:
                                self.BlueDestroyerInfo[1] = 0
                else:
                    print("no ship is detected!")
            if len(self.BlueDestroyerInfoList)>0:
                print("cur_time=",cur_time,"self.BlueDestroyerInfoList=",self.BlueDestroyerInfoList)

    #更新攻击蓝方舰艇的攻击表
    def update_A2CommandAttackList(self,cur_simtime):
        del_count = 0
        for i in range(len(self.A2Command_AttackList)):
            #当前仿真时间与导弹发射
            # 时间的差与导弹预估飞行时间
            a2s_record = self.A2Command_AttackList[i-del_count]
            if (cur_simtime - a2s_record[0]) > (a2s_record[3]+120):
                del self.A2Command_AttackList[i-del_count]
                del_count += 1

    #新版本 更新对蓝方目标实施攻击的轰炸机存活情况和弹药情况
    def update_A2GPCBomerListInfo(self, dict_units, a2GPCBomerList):
        #攻击蓝方zhihisuo的轰炸机的存活情况 1-存活 0-被击落
        existedflag = []
        #攻击蓝方舰艇的轰炸机的弹药情况 -1表示被击落的轰炸机弹药
        bomermoutnum_list = []

        if len(a2GPCBomerList)>0:
            for i in range(len(a2GPCBomerList)):
                bomerID = a2GPCBomerList[i]
                iflag, member_state = Dict_GetUnitStateByID(dict_units, bomerID)
                if iflag:
                    existedflag.append(1)
                    mflag, member_mountnum = Dict_GetUnitMunNumByID(dict_units, bomerID)
                    if mflag:
                        bomermoutnum_list.append(member_mountnum)
                else:
                    existedflag.append(0)
                    bomermoutnum_list.append(-1)

        return existedflag, bomermoutnum_list

    #得到蓝方指挥所ID
    def GetCommandInfo(self,dict_qb):
        eflag, commandIDList = Dict_GetQBUnitsID_ByType(dict_qb, UnitType.COMMAND, 0)
        if eflag:
            if self.ObtaincommandInfo == False:
                for cmdID in commandIDList:
                    iflag, commpos = Dict_GetUnitPosByID(dict_qb, cmdID, 2)
                    if iflag:
                        alive = 0
                        flag, enemy_info = Dict_GetUnitInfo(dict_qb, cmdID)
                        if flag:
                            alive = enemy_info.isAlive

                        if commpos[1] > 0:
                            self.BlueNorthCommandInfo.append(cmdID)
                            self.BlueNorthCommandInfo.append(alive)
                            self.BlueNorthCommandInfo.append(commpos)
                        else:
                            self.BlueSouthCommandInfo.append(cmdID)
                            self.BlueSouthCommandInfo.append(alive)
                            self.BlueSouthCommandInfo.append(commpos)
                #生成初始攻击列表
                self.AttackPriorityList.append(['S', self.BlueSouthCommandInfo, False])
                self.AttackPriorityList.append(['N',self.BlueNorthCommandInfo,False])
                self.ObtaincommandInfo = True

            if self.ObtaincommandInfo == True:
                if len(commandIDList)==1:
                    if commandIDList[0] == self.BlueSouthCommandInfo[0]:
                        self.BlueNorthCommandInfo[1] = 0
                    else:
                        self.BlueSouthCommandInfo[1] = 0
        else:
            self.BlueNorthCommandInfo[1]=0
            self.BlueSouthCommandInfo[1]=0
        print("北岛指挥所=",self.BlueNorthCommandInfo,"南岛指挥所=",self.BlueSouthCommandInfo)

    #攻击蓝方地面/水面目标  进攻北部(NSflag='N') 进攻南部(NSflag='S')- 进攻舰艇(NSflag='D')改进版
    def Attack_BlueSurfaceTarget(self, obs_red, dict_units, dict_qb, NSflag,cur_time):
        a2NGCcmdlist = []
        A2CBlueSurfaceTargetInfo = []

        if NSflag == 'N':
            A2CBlueSurfaceTargetInfo = self.BlueNorthCommandInfo
            self.A2C_ObjectFlag = 1
        if NSflag == 'S':
            A2CBlueSurfaceTargetInfo = self.BlueSouthCommandInfo
            self.A2C_ObjectFlag = 2
        if NSflag == 'D':
            A2CBlueSurfaceTargetInfo = self.BlueDestroyerInfo
            self.A2C_ObjectFlag = 3

        #print("打击的蓝方地面/水面目标信息=",NSflag, A2CBlueSurfaceTargetInfo)
        SurfTarID = A2CBlueSurfaceTargetInfo[0]
        SurfTarAlive = A2CBlueSurfaceTargetInfo[1]
        SurfTarPos = A2CBlueSurfaceTargetInfo[2]

        if self.opration_stage == RedAgentOperationStage.Mass_Force:
            return []

        if len(self.A2GCP_BomerList)==0 and (self.opration_stage == RedAgentOperationStage.Formation_Navaigation):
            #print("攻击蓝方地面/水面目标时的Agent状态",self.agent_state)
            # 1-1 计算出大编队攻击指挥所的待机点
            FormA2GCStandby_Pos = CalcDesPtByAzRadius(SurfTarPos, 90, FORMATION_TO_A2STARGETDIST)
            # 1-2 下达计划航线
            flag, center_pos = Dict_GetUnitPosByID(dict_units, self.Form_CenterID, 3)
            if flag:
                if self.JammerEvadeState == False:
                    a2NGCcmdlist.extend(self._aircraft_areapatrol(self.Form_CenterID,
                                                              [FormA2GCStandby_Pos[0], FormA2GCStandby_Pos[1], 7500],
                                                              AWACS_PATROL_PARAMS))
                    self.Form_CurrNavPos = FormA2GCStandby_Pos
            #1-3 到达待机点
            _,dist = CalcP2PDistance(self.Form_CenterPos, FormA2GCStandby_Pos, 2)
            if dist< 15000:
                self.agent_state = RedAgentState.COMMAND_ATTACK_MISSIONASSIGN
                self.opration_stage = RedAgentOperationStage.Attack_Stage
                print(cur_time,"秒,进入进攻蓝方地面/水面目标阶段")

        #攻击任务分配
        if len(self.A2GCP_BomerList)==0 and self.opration_stage == RedAgentOperationStage.Attack_Stage\
                                and self.agent_state == RedAgentState.COMMAND_ATTACK_MISSIONASSIGN:

            iflag, bomerIDList = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.A2G)
            if iflag:
                bomerIDList = list(set(bomerIDList) - set([self.Form_CenterID]))   # 如果编队中心是轰炸机则去掉它
                _, bomerMuntNumList = Dict_GetUnitsMunNumByIDList(dict_units, bomerIDList)
                #空中有弹的飞机
                bomerCanA2GList = [bomerIDList[i] for i in range(len(bomerIDList)) if bomerMuntNumList[i]>0]
                # #空中没弹的飞机
                # bomerNoMuntList = list(set(bomerIDList) - set(bomerCanA2GList))
                # 机场待战的数量
                airport_bomernum = self.GetAirportBomerNum(obs_red)

                if (len(bomerIDList)+airport_bomernum)>=BomerA2GNumInSametime:
                    reqa2gBomerNum = BomerA2GNumInSametime
                else:
                    reqa2gBomerNum = len(bomerIDList)+airport_bomernum

                _, bomerCanA2GPosList = Dict_GetUnitsPosByIDList(dict_units, bomerCanA2GList, 2)
                BomerCanA2GToCenterdist = [(bomerCanA2GList[i], CalcP2PDistance(bomerCanA2GPosList[i], self.Form_CenterPos, 2)[1]) for i in range(len(bomerCanA2GList))]
                BomerCanA2GToCenterdist = sorted(BomerCanA2GToCenterdist, key=lambda kv: (kv[1], kv[0]))

                BomerMassOverflag = [0]*len(BomerCanA2GToCenterdist)
                for i in range(len(BomerCanA2GToCenterdist)):
                    toCenterdist = BomerCanA2GToCenterdist[i][1]
                    if toCenterdist < 20000:
                        BomerMassOverflag[i] = 1
                    else:
                        BomerMassOverflag[i] = 0
                AirA2GBomerNum = sum(BomerMassOverflag)

                if AirA2GBomerNum>=reqa2gBomerNum:
                    self.A2GCP_BomerList =[BomerCanA2GToCenterdist[i][0] for i in range(len(BomerCanA2GToCenterdist)) if i<reqa2gBomerNum]

            print("可参与攻击蓝方地面/水面目标的轰炸机列表=", self.A2GCP_BomerList)

        #分配攻击任务的轰炸机占领阵位
        if  len(self.A2GCP_BomerList)>0 and self.OcupyA2GCPPosManuCmdExed == False:
            # 1-1 计算出大编队中心距离敌方指挥所距离方位
            CentertoCommandposAz = CalcAzByTwoPts(SurfTarPos, self.Form_CenterPos)
            _, CentertoCommandpos = CalcP2PDistance(SurfTarPos, self.Form_CenterPos, 2)
            CentertoCommandposAz = 90
            print("大编队攻击待机点中心距离蓝方地面/水面目标时距离和方位=", [CentertoCommandpos, CentertoCommandposAz])
            # 1-2 得到攻击方位角度
            center_A2GAngle = int((CentertoCommandposAz + 180) % 360)
            #新版本  攻击方位角度列表
            for i in range(len(self.A2GCP_BomerList)):
                if i % 2 == 0:
                    delta = -(i*3 / 2)
                else:
                    delta = 3*(i + 1)/2
                singleA2GCPAngle = (center_A2GAngle + delta) if (center_A2GAngle + delta)<360 else (center_A2GAngle + delta)%360
                self.A2GCP_BomerA2GAngle.append(singleA2GCPAngle)
            print("攻击角度表=",self.A2GCP_BomerA2GAngle)

            # 1-4 计算出轰炸机的攻击位置
            self.A2NGC_Pos = CalcDesPtByAzRadius(SurfTarPos, CentertoCommandposAz, A2S_AttackDistance)
            # 新版本 轰炸机攻击阵位的方位角度列表
            A2GCP_BomerAzAngleList = [sa2gAngle-180 for sa2gAngle in self.A2GCP_BomerA2GAngle]
            # 新版本 轰炸机占攻击阵位
            A2GCP_BomerAzDistList = [[A2GCP_BomerAzAngleList[i], A2S_AttackDistance ] for i in range(len(self.A2GCP_BomerList))]
            self.A2GCP_BomerA2GPos, _ = self.compute_formation_abspos(SurfTarPos, A2GCP_BomerAzDistList,270)
            #print("轰炸机攻击位置表=",self.A2GCP_BomerA2GPos)
            _, BomerPosList = Dict_GetUnitsPosByIDList(dict_units, self.A2GCP_BomerList, 2)
            a2gcp_posassign_list = minMaxWeightMatchingCppVer(self.A2GCP_BomerA2GPos, BomerPosList)
            #a2gcp_posassign_list = sorted(a2gcp_posassign_list, key=lambda x: (x[0], x[1]))
            #print("轰炸机攻击位置分配表=", a2gcp_posassign_list)
            JudgeBomerList = []
            for Serve_Tuple in a2gcp_posassign_list:
                if Serve_Tuple[1] == -1:
                    continue
                bomer_id = self.A2GCP_BomerList[Serve_Tuple[1]]
                JudgeBomerList.append(bomer_id)
                i_flag, bomer_pos = Dict_GetUnitPosByID(dict_units, bomer_id, 3)
                _     , bomer_spd = Dict_GetUnitSpeedByID(dict_units, bomer_id)
                _     , bomer_heanding = Dict_GetUnitHeadingByID(dict_units, bomer_id)
                if i_flag:
                    fcur_pos = bomer_pos
                    formationspd_vector = CalcEdPosByDisAngle(bomer_spd, bomer_heanding)
                    dist_vector = [(self.A2GCP_BomerA2GPos[Serve_Tuple[0]][0] - fcur_pos[0]),
                                   (self.A2GCP_BomerA2GPos[Serve_Tuple[0]][1] - fcur_pos[1])]
                    opt_spdvector = optimalSpeed(list(formationspd_vector), 220, dist_vector)
                    opt_speed = lengthofVector(opt_spdvector)
                    if (opt_speed <= 220):
                        opt_speed = 220
                    line_params = [
                        Point(fcur_pos[0] + opt_spdvector[0] * 5, fcur_pos[1] + opt_spdvector[1] * 5, fcur_pos[2]),
                        Point(self.A2GCP_BomerA2GPos[Serve_Tuple[0]][0], self.A2GCP_BomerA2GPos[Serve_Tuple[0]][1], fcur_pos[2])]
                    a2NGCcmdlist.extend(self._aircraft_linepatrol(bomer_id, opt_speed, line_params))
            self.A2GCP_BomerList = JudgeBomerList
            #print("分配攻击位置后的轰炸机列表=", self.A2GCP_BomerList)
            self.A2GCP_BomerAttackcmdConfirm = [0 for i in range(len(self.A2GCP_BomerList))]
            self.A2GCP_BomerAtgMissleLauched = [0 for i in range(len(self.A2GCP_BomerList))]
            self.OcupyA2GCPPosManuCmdExed = True

        if len(self.A2GCP_BomerList) > 0 and self.OcupyA2GCPPosManuCmdExed == True and self.AttackCommandCmdExcuted == False:
            # 派出离目标最近的同样架数的战斗机护航(单架战斗机器不能护航)
            if self.EcovyFighterManucmdExed == False:
                self.A2GCP_EncoyFighter = []
                reqecovFighernum = len(self.A2GCP_BomerList)
                iflag, figherIDList = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.A2A)
                _, fighterMuntNumList = Dict_GetUnitsMunNumByIDList(dict_units, figherIDList)
                if len(fighterMuntNumList) < reqecovFighernum:
                    reqecovFighernum = len(fighterMuntNumList)
                FighterCanA2AList = [figherIDList[i] for i in range(reqecovFighernum) if fighterMuntNumList[i] > 0]  # 空中有弹的飞机
                if len(FighterCanA2AList)>0:
                    flag, rel_dist_list = Dict_CalcRelDistByTwoUnitIDList(dict_qb, dict_units, [SurfTarID], FighterCanA2AList, 2)
                    if flag and len(rel_dist_list)>0:
                        rel_dist_list = sorted(rel_dist_list, key=lambda x: (x[0], x[1]))
                        FighterCanEncovyList = [rel_dist_list[index][1][1] for index in range(len(rel_dist_list))]
                        print("前出护航的战斗机=",FighterCanEncovyList)
                        # 按离目标最近的同样数量的战斗机
                        if len(FighterCanEncovyList)>0:
                            flag , bomerPos = Dict_GetUnitPosByID(dict_units, self.A2GCP_BomerList[0], 2)
                            if flag:
                                _,  tardist = CalcP2PDistance(SurfTarPos, self.A2GCP_BomerA2GPos[0], 2)
                                if tardist<115000:
                                    for j in range(len(FighterCanEncovyList)):
                                        _, fTmid = Dict_GetUnitTeamIDByID(dict_units, FighterCanA2AList[j])
                                        _, bTmid = Dict_GetUnitTeamIDByID(dict_units, self.A2GCP_BomerList[j])
                                        a2NGCcmdlist.extend(self._A2G_airescort(fTmid, bTmid))
                                        self.A2GCP_EncoyFighter.append(FighterCanA2AList[j])
                                    self.EcovyFighterManucmdExed = True

            # 新版本 轰炸机对目标发起攻击
            _, bomermuntlist = Dict_GetUnitsMunNumByIDList(dict_units, self.A2GCP_BomerList)

            for i in range(len(self.A2GCP_BomerList)):
                bomerID = self.A2GCP_BomerList[i]
                flag , bomerPos = Dict_GetUnitPosByID(dict_units, bomerID, 2)
                if flag:
                    _,  A2GCPdist = CalcP2PDistance(bomerPos, self.A2GCP_BomerA2GPos[i], 2)
                    if bomermuntlist[i]>0 and A2GCPdist<4500 and self.A2GCP_BomerAttackcmdConfirm[i]==0 and SurfTarAlive:
                        a2NGCcmdlist.extend(self._targethunt(bomerID, SurfTarID, self.A2GCP_BomerA2GAngle[i],int(A2S_AttackDistance * 100 / A2S_MAXAttackRange)))
                        print("bomer=",bomerID,"SurfTarID=",SurfTarID,"攻击角=",self.A2GCP_BomerA2GAngle[i],"攻击距离百分比=",int(A2S_AttackDistance * 100 / A2S_MAXAttackRange))
                        self.A2GCP_BomerAttackcmdConfirm[i] = 1
                else:
                    self.A2GCP_BomerAttackcmdConfirm[i] = 1

            for i in range(len(self.A2GCP_BomerList)):
                if bomermuntlist[i] is not None and self.A2GCP_BomerAttackcmdConfirm[i]==1:
                    if bomermuntlist[i]<self.A2GCP_BomerMnulist[i] and SurfTarAlive:
                        flag, bomerPos = Dict_GetUnitPosByID(dict_units, self.A2GCP_BomerList[i], 2)
                        _, A2GCPdist = CalcP2PDistance(bomerPos, SurfTarPos, 2)
                        a2NGCcmdlist.extend(self._targethunt(self.A2GCP_BomerList[i], SurfTarID, self.A2GCP_BomerA2GAngle[i],
                                                             int(min(A2S_AttackDistance,A2GCPdist-1500) * 100 / A2S_MAXAttackRange)))
                        self.A2Command_AttackList.append([cur_time, SurfTarID, self.A2GCP_BomerList[i], self.predict_milssleflytime(obs_red, SurfTarID, self.A2GCP_BomerList[i])])
                        print("时间=",cur_time,"轰炸机=",SurfTarID,"对目标已发起攻击")

            print("攻击轰炸机当前弹药量",bomermuntlist)
            print("攻击预测表=",self.A2Command_AttackList)
            self.A2GCP_BomerMnulist = bomermuntlist
            if sum(ni for ni in bomermuntlist if ni is not None)==0:
                self.agent_state = RedAgentState.COMMAND_ATTACK_CMDEXCUTED
                self.AttackCommandCmdExcuted = True

        return a2NGCcmdlist

    #判断蓝方地面/水面目标是否攻击结束-改进版
    def JudgeBlueSurfaceTargetAttackEnd(self,dict_units,cur_time,Cflag):

        if self.AttackCommandCmdExcuted == True and self.agent_state == RedAgentState.COMMAND_ATTACK_CMDEXCUTED:
            if Cflag == 'N':
                SurfTarAlive = self.BlueNorthCommandInfo[1]
            if Cflag == 'S':
                SurfTarAlive = self.BlueSouthCommandInfo[1]
            if Cflag == 'D':
                SurfTarAlive = self.BlueDestroyerInfo[1]

            if SurfTarAlive==0:
                print("时间=", cur_time, "蓝方目标打掉了,任务完成！")
                self.A2GCP_BomerList.clear()
                self.A2GCP_EncoyFighter.clear()
                self.EcovyFighterManucmdExed = False
                self.AttackCommandCmdExcuted = False
                self.OcupyA2GCPPosManuCmdExed = False
                for i in range(len(self.AttackPriorityList)):
                    astarInfo = self.AttackPriorityList[i]
                    if astarInfo[0] == Cflag:
                        self.AttackPriorityList[i][2] = True
                if len(self.AttackPriorityList)>0:
                    self.opration_stage = RedAgentOperationStage.Formation_Navaigation
                else:
                    self.agent_state = RedAgentState.COMMAND_ATTACK_MISSONCOMPLETED
                return True
            else:
                self.update_A2CommandAttackList(cur_time)
                if len(self.A2Command_AttackList) == 0:
                    # 取消攻击编组,重新选择编组进行攻击
                    self.A2GCP_BomerList.clear()
                    self.A2GCP_EncoyFighter.clear()
                    self.EcovyFighterManucmdExed = False
                    self.AttackCommandCmdExcuted = False
                    self.OcupyA2GCPPosManuCmdExed = False
                    self.agent_state = RedAgentState.COMMAND_ATTACK_MISSIONASSIGN
                else:
                    print("当前时间=",cur_time,"正在攻击蓝方水/地面目标中...", Cflag)
                    print("导弹攻击预测表", self.A2Command_AttackList)
                    Baliveflag, Bmountlist = self.update_A2GPCBomerListInfo(dict_units, self.A2GCP_BomerList)
                    print("当前时间=",cur_time,"攻击实体=",self.A2GCP_BomerList)
                    print("当前时间=",cur_time,"存活情况=",Baliveflag,"弹药数量=",Bmountlist)
                return False

    #计算实体到达编队中期望位置点的最优速度及其速度矢量
    def CalArriveExpPosOptimalSpd(self,formSpd, formHeanding, CurrPos, ExpectPos,EntMaxSpd,EntMinSpd):
        #编队中心点的速度矢量
        formationspd_vector = CalcEdPosByDisAngle(formSpd, formHeanding)
        # 当前位置到期望位置的距离矢量
        dist_vector = [(ExpectPos[0] - CurrPos[0]), (ExpectPos[1] - CurrPos[1])]
        opt_spdvector = optimalSpeed(list(formationspd_vector), EntMaxSpd, dist_vector)
        opt_speed = lengthofVector(opt_spdvector)
        if (opt_speed <= EntMinSpd):
            opt_speed = EntMinSpd
        return opt_speed,opt_spdvector

    #机场起飞轰炸机控制
    def AirportControl(self,obs_red,dict_units, cur_time):
        airportcmd = []
        _, figherlist = Dict_GetOwnUnitsID_ByType(dict_units,UnitType.A2A)
        _, bomerlist = Dict_GetOwnUnitsID_ByType(dict_units,UnitType.A2G)
        _, bomermunlist = Dict_GetUnitsMunNumByIDList(dict_units, bomerlist)
        bomer_CanA2Glist = [bomerlist[i] for i in range(len(bomermunlist)) if bomermunlist[i]>0]
        for redunit in obs_red['airports']:
            if redunit['ID'] == 30001:
                if redunit['BOM'] >= 1 and cur_time >= 1800:
                    airportcmd.extend(
                        self._takeoff_areapatrol(1, UnitType.A2G, Awacs_Mass_Point, Fighter_MassArea_Params))
                if redunit['AIR'] >=1 and cur_time >= 1800:
                    airportcmd.extend(
                        self._takeoff_areapatrol(1, UnitType.A2A, Awacs_Mass_Point, Fighter_MassArea_Params))
        return airportcmd

    def GetAirportBomerNum(self,obs_red):
        airport_bomernum = 0
        for redunit in obs_red['airports']:
            if redunit['ID'] == 30001:
               airport_bomernum = redunit['BOM']
        return airport_bomernum

    #如果一个步长内同一实体既有机动指令又有攻击指令,保留攻击指令
    def ManuCmdA2ACmdOptimal(self,ManuCmdList, A2ACmdList):

        ManuCmdDelEntIDList = [ ManuCmdDict['self_id']  for ManuCmdDict in ManuCmdList for A2ACmdDict in A2ACmdList if
                            ManuCmdDict['self_id'] == A2ACmdDict['self_id'] ]
        #print("需要删除机动命令的实体列表",ManuCmdDelEntIDList)
        ManuCmdDelEntIDList = list(set(ManuCmdDelEntIDList))
        del_count = 0
        for i in range(len(ManuCmdDelEntIDList)):
            #当前仿真时间与导弹发射时间的差与导弹预估飞行时间
            ManuCmdDict = ManuCmdList[i-del_count]
            if ManuCmdDict['self_id'] == ManuCmdDelEntIDList[i]:
                del ManuCmdList[i-del_count]
                del_count += 1

    # add by lq
    def air_defense_planning2(self, dict_units, dict_qb, sim_time):
        #print('current air targets')
        #print(self.blue_air_ids_current)
        cmd_list = []
        assign_df =  self.air_defense_assign_info_history
        # columns = ['target_id', 'target_type', 'air_defense_id', 'air_defense_type', 'start_time', 'end_time']
        # judge idle channels
        # ad_df =  self.own_air_defense_df
        air_defense_ids = self.own_air_defense_df['air_defense_id'].tolist()

        # delete history info
        if len(assign_df)>0:
            assign_df = assign_df.drop(assign_df[assign_df.end_time <= sim_time].index)

        # calculate idle channels
        for air_defense_id in air_defense_ids:
            # get air defense mun number and type
            _, mun_num = Dict_GetUnitMunNumByID(dict_units, air_defense_id)
            ad_type = self.own_air_defense_df[self.own_air_defense_df.air_defense_id == air_defense_id]['air_defense_type'].tolist()[0]

            # calculation occupied channels
            items = assign_df[(assign_df.air_defense_id == air_defense_id) & (assign_df.end_time > sim_time)]['air_defense_type'].tolist()

            if ad_type == UnitType.SHIP:
                self.own_air_defense_df.loc[self.own_air_defense_df['air_defense_id'] == air_defense_id, ['idle_channel_num']] = [min(mun_num, AirDefenseMaxChannel.SHIP_CHANNEL - len(items))]
            else:
                self.own_air_defense_df.loc[self.own_air_defense_df['air_defense_id'] == air_defense_id, ['idle_channel_num']] = [min(mun_num, AirDefenseMaxChannel.S2A_CHANNEL - len(items))]

        if np.sum(self.own_air_defense_df['idle_channel_num'].values) > 0:

            # filter targets
            if len(self.blue_air_ids_current) > 0:
                # if len(assign_df)>0:
                #     print('aaa')
                new_targets = air_defense_filter_targets(dict_units, air_defense_ids, dict_qb, self.blue_air_ids_current, assign_df, sim_time)

                if len(new_targets) > 0:
                    new_target_assign, self.own_air_defense_df = air_defense_assignment(self.own_air_defense_df , new_targets)
                    print(len(new_target_assign))
                    print(self.own_air_defense_df)

                    # form cmd
                    for index, row in new_target_assign.iterrows():
                        cmd_list.extend(self.air_defense_attack(row['air_defense_id'], row['air_defense_type'],
                                                                row['target_id']))
                    self.air_defense_assign_info_history = pd.concat([assign_df, new_target_assign], axis=0)

                    print(self.air_defense_assign_info_history[['air_defense_id', 'target_id', 'start_time', 'end_time']])

        return cmd_list

    def air_defense_planning(self, dict_units, dict_qb, sim_time):
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
            _, mun_num = Dict_GetUnitMunNumByID(dict_units, k)

            if v.Type == UnitType.SHIP:
                current_idle_channels_num = AirDefenseMaxChannel.SHIP_CHANNEL - v.UsedChannelNum
                v.IdleChannelNum = min(mun_num, current_idle_channels_num)
            else:
                current_idle_channels_num = AirDefenseMaxChannel.S2A_CHANNEL - v.UsedChannelNum
                v.IdleChannelNum = min(mun_num, current_idle_channels_num)

        # print('air_defense_info:')
        # for (k, v) in self.own_air_defense_dict.items():
        #     print('air_defense_id: %d type: %d idle_channel_num: %d used_channel_num:%d' % (k, v.Type, v.IdleChannelNum, v.UsedChannelNum))

        if len(self.blue_air_ids_current) > 0:
            new_targets = dict_air_defense_filter_targets(dict_units, list(self.own_air_defense_dict.keys()), dict_qb, self.blue_air_ids_current, assign_dict, sim_time)

            if len(new_targets) > 0:
                start = time.time()
                new_target_assign = dict_air_defense_assignment(self.own_air_defense_dict, new_targets, sim_time, True)
                end = time.time()
                print('Blue Side air_defense_assignment Running time: %s Seconds' % (end - start))

                # form cmd
                for (k, v) in new_target_assign.items():
                    _, air_defense_type = Dict_GetUnitTypeByID(dict_units, v.air_defense_id)
                    cmd_list.extend(self.air_defense_attack(v.air_defense_id, air_defense_type, k))

                # Merge
                self.air_defense_assign_history_dict.update(new_target_assign)

        return cmd_list

    def air_defense_planning4(self, dict_units, dict_qb, sim_time):
        cmd_list = []
        red_fighter = []
        red_awacs = []
        red_jammer = []
        red_bomber = []
        red_uav = []

        for blue_air_id in self.blue_air_ids_current:
            _, target_type = Dict_GetUnitTypeByID(dict_qb, blue_air_id)

            if target_type == UnitType.A2A:
                red_fighter.append(blue_air_id)
            elif target_type == UnitType.A2G:
                red_bomber.append(blue_air_id)
            elif target_type == UnitType.AWACS:
                red_awacs.append(blue_air_id)
            elif target_type == UnitType.DISTURB:
                red_jammer.append(blue_air_id)
            elif target_type == UnitType.UAV:
                red_uav.append(blue_air_id)
            else:
                print('unknown')

        # for (k, v) in self.aircrafts_info_current.items():
        #     if k == UnitType.A2A:
        #         red_fighter.extend(v)
        #     elif k == UnitType.A2G:
        #         red_bomber.extend(v)
        #     elif k == UnitType.AWACS:
        #         red_awacs.extend(v)
        #     elif k == UnitType.DISTURB:
        #         red_jammer.extend(v)
        #     elif k == UnitType.UAV:
        #         red_uav.extend(v)
        #     else:
        #         print('unknown')

        for (k, v) in self.own_air_defense_dict.items():
            if v.Type == UnitType.SHIP:
                for uav_id in red_uav:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, uav_id, 2)
                    if dist <= 100000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, uav_id))

                for jammer_id in red_jammer:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, jammer_id, 2)
                    if dist <= 100000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, jammer_id))


                for awacs_id in red_awacs:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, awacs_id, 2)
                    if dist <= 100000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, awacs_id))

                for bommer_id in red_bomber:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, bommer_id, 2)
                    if dist <= 90000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, bommer_id))

                for fighter_id in red_fighter:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, fighter_id, 2)
                    if dist <= 75000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, fighter_id))
            else:
                for jammer_id in red_jammer:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, jammer_id, 2)
                    if dist <= 100000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, jammer_id))

                for awacs_id in red_awacs:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, awacs_id, 2)
                    if dist <= 100000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, awacs_id))

                for bommer_id in red_bomber:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, bommer_id, 2)
                    if dist <= 90000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, bommer_id))

                for fighter_id in red_fighter:
                    _, dist = Dict_CalcDistByTwoUnitID(dict_units, dict_qb, k, fighter_id, 2)
                    if dist <= 80000:
                        cmd_list.extend(self.air_defense_attack(k, v.Type, fighter_id))

        return cmd_list

    # s2a
    def air_defense_attack(self, self_id, self_type, target_id):
        if self_type == UnitType.S2A:
            return [EnvCmd.make_ground_addtarget(self_id, target_id)]
        if self_type == UnitType.SHIP:
            return [EnvCmd.make_ship_addtarget(self_id, target_id)]


    def air_defense_init2(self,dict_units):
        _, ship_id_list = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.SHIP)

        self.own_ship_first_id = ship_id_list[0]
        _, first_ship_mun_num = Dict_GetUnitMunNumByID(dict_units, self.own_ship_first_id)

        self.own_ship_second_id = ship_id_list[1]
        _, second_ship_mun_num = Dict_GetUnitMunNumByID(dict_units, self.own_ship_second_id)

        self.own_air_defense_df = self.own_air_defense_df.append([{'air_defense_id': self.own_ship_first_id, 'air_defense_type': UnitType.SHIP,
                                                                   'idle_channel_num': 4, 'current_mun_num':first_ship_mun_num}], ignore_index= True)

        self.own_air_defense_df = self.own_air_defense_df.append([{'air_defense_id': self.own_ship_second_id, 'air_defense_type': UnitType.SHIP,
                                                                   'idle_channel_num': 4, 'current_mun_num':second_ship_mun_num}], ignore_index= True)

        return

    def air_defense_init(self, dict_units):
        _, ship_id_list = Dict_GetOwnUnitsID_ByType(dict_units, UnitType.SHIP)

        self.own_ship_first_id = ship_id_list[0]
        _, first_ship_mun_num = Dict_GetUnitMunNumByID(dict_units, self.own_ship_first_id)

        self.own_ship_second_id = ship_id_list[1]
        _, second_ship_mun_num = Dict_GetUnitMunNumByID(dict_units, self.own_ship_second_id)

        self.own_air_defense_dict[self.own_ship_first_id] = AirDefenseInfo(UnitType.SHIP, 4, 0)
        self.own_air_defense_dict[self.own_ship_second_id] = AirDefenseInfo(UnitType.SHIP, 4, 0)


        return
    # def _s2a_attack_cancel(self, self_id, self_type, target_id):
    #     if self_type == UnitType.S2A:
    #         return [EnvCmd.make_ground_removetarget(self_id, target_id)]
    #     if self_type == UnitType.SHIP:
    #         return [EnvCmd.make_ship_removetarget(self_id, target_id)]
    #
    # def fire_control(self, self_id, target_id, fire_range):
    #     fire_flag = False
    #     fire_cmd = []
    #     _, dist = CalcDistByTwoUnitID(self.dict_units, self.dict_qb, self_id, target_id, 2)
    #
    #     _, self_type = GetUnitTypeByID(self.dict_units, self_id)
    #     print('target_id:%d' % target_id)
    #     print('dist:%f' % dist)
    #     if dist <= fire_range:
    #         fire_cmd.extend(self._s2a_attack(self_id, self_type[0], target_id))
    #         fire_flag = True
    #
    #     return fire_flag, fire_cmd

    def air_defense_update2(self, dict_units):
        first_ship_flag, first_ship_type = GetUnitTypeByID(dict_units, self.own_ship_first_id)

        if not first_ship_flag:
            self.own_air_defense_df = self.own_air_defense_df.drop(self.own_air_defense_df[self.own_air_defense_df.air_defense_id == self.own_ship_first_id].index)

        second_ship_flag, second_ship_s2a_type = GetUnitTypeByID(dict_units,self.own_ship_second_id)

        if not second_ship_flag:
            self.own_air_defense_df = self.own_air_defense_df.drop(self.own_air_defense_df[self.own_air_defense_df.air_defense_id == self.own_ship_second_id].index)

    def air_defense_update(self, dict_units):
        first_ship_flag, first_ship_type = Dict_GetUnitTypeByID(dict_units, self.own_ship_first_id)

        if not first_ship_flag and self.own_ship_first_id in self.own_air_defense_dict:
            del self.own_air_defense_dict[self.own_ship_first_id]

        second_ship_flag, second_ship_s2a_type = Dict_GetUnitTypeByID(dict_units, self.own_ship_second_id)

        if not second_ship_flag and self.own_ship_second_id in self.own_air_defense_dict:
            del self.own_air_defense_dict[self.own_ship_second_id]



    # 航线巡逻[wt增加]
    @staticmethod
    def _aircraft_areapatrol(aircraft_id, areapatrol_point, areapatrol_params):
        return [EnvCmd.make_areapatrol(aircraft_id,*areapatrol_point,*areapatrol_params)]

    # 航线巡逻[wt增加]
    @staticmethod
    def _aircraft_linepatrol(aircraft_id,patrol_spd,line_params):
        return [EnvCmd.make_linepatrol(aircraft_id,patrol_spd,1,'line',line_params)]

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
        return [EnvCmd.make_disturb_linepatrol(self_id, 200, 0, 'line', point_list)]

    # 轰炸机起飞突击
    @staticmethod
    def _takeoff_areahunt(num, area_hunt_point):
        return [EnvCmd.make_takeoff_areahunt(RED_AIRPORT_ID, num, 270, 80, *area_hunt_point, *[270, 1000, 1000, 160])]

    # 干扰机护航
    def _disturb_escort(self, disturb_team_id):
        return [EnvCmd.make_takeoff_protect(RED_AIRPORT_ID, 2, disturb_team_id, 1, 100, 250)]

    # 轰炸机起飞护航
    def _A2G_escort(self, a2g_team_id):
        return [EnvCmd.make_takeoff_protect(RED_AIRPORT_ID, 2, a2g_team_id, 1, 100, 250)]

    # 轰炸机护航
    def _A2G_airescort(self,self_id, bomer_id):
        return [EnvCmd.make_protect( self_id, bomer_id, 1, 100)]

    # 起飞区域巡逻
    @staticmethod
    def _takeoff_areapatrol(num, lx, patrol_point, patrol_params):
        # patrol_params为5个参数
        return [EnvCmd.make_takeoff_areapatrol(RED_AIRPORT_ID, num, lx, *patrol_point, *patrol_params)]

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
        return [EnvCmd.make_returntobase(unit_id, 30001)]

    # 轰炸机目标突击
    @staticmethod
    def _targethunt(self_id, target_id, a2saz, a2srange):
        return [EnvCmd.make_targethunt(self_id, target_id, a2saz, a2srange)]

    # 轰炸机区域突击
    @staticmethod
    def _areahunt(self_id, a2sangle, RangeRatio, point):
        return [EnvCmd.make_areahunt(self_id, a2sangle, RangeRatio, *point, *AREA_HUNT_PARAMS)]

    # 护卫舰区域巡逻
    def _ship_areapatrol(self, self_id, point):
        return [EnvCmd.make_ship_areapatrol(self_id, *point, *SHIP_PATROL_PARAMS_0)]

    # 护卫舰初始化部署
    def _ship_movedeploy(self, self_id, point):
        return [EnvCmd.make_ship_movedeploy(self_id, *point, 90, 1)]

    def s2a_power_off(self, self_id):
        return [EnvCmd.make_ground_radarcontrol(self_id, 0)]

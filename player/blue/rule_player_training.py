from env.env_def import UnitType, BLUE_AIRPORT_ID, MapInfo
from common.cmd import Command
from common.grid import MapGrid
from common.interface.base_rule import BaseRulePlayer
from common.interface.task import Task, TaskState
from player.agent_util_modifiy import *
from env.env_cmd import EnvCmd
from agent.DeepBlue.player.agent import Agent
# import numpy as np
import math

NORTH_COMMAND_POS = [-129532, 87667, 0]
SOUTH_COMMAND_POS = [-131154, -87888, 0]
NEW_NORTH_COMMAND_POS = [-250830, 184288, 0]
NEW_SOUTH_COMMAND_POS = [-295519, -100815, 0]
NORTH_FIRST_S2A_POS = [-232595, 182906, 0]
NORTH_SECOND_S2A_POS = [-232595, 192906, 0]
SOUTH_FIRST_S2A_POS = [-285888, -79738, 0]
SOUTH_SECOND_S2A_POS = [-285888, -80738, 0]

Fighter_MassArea_Params = [270, 5000, 5000, 1000 / 3.61, 7200]
# 0910
SHIP_TO_CMD_DIS = 45000
SHIP_TO_CMD_ANGLE = -30 * math.pi / 180
Fighter_offset = 45000
FIGHTER_TO_SHIP_ANGLE = -30*math.pi/180
FIRST_SHIP_POINT = [NEW_NORTH_COMMAND_POS[0] + SHIP_TO_CMD_DIS * math.cos(SHIP_TO_CMD_ANGLE), NEW_NORTH_COMMAND_POS[1] + SHIP_TO_CMD_DIS * math.sin(SHIP_TO_CMD_ANGLE), 0]
# FIRST_SHIP_POINT = NEW_NORTH_COMMAND_POS
SECOND_SHIP_POINT = [NEW_SOUTH_COMMAND_POS[0] + SHIP_TO_CMD_DIS * math.cos(-SHIP_TO_CMD_ANGLE), NEW_SOUTH_COMMAND_POS[1] + SHIP_TO_CMD_DIS * math.sin(-SHIP_TO_CMD_ANGLE), 0]
Fighter_Near_South_Ship_Pos = [SECOND_SHIP_POINT[0] + Fighter_offset * math.cos(-FIGHTER_TO_SHIP_ANGLE), SECOND_SHIP_POINT[1] + Fighter_offset * math.sin(-FIGHTER_TO_SHIP_ANGLE), 7500]
A2A_PATROL_PARAMS = [270, 10000, 10000, 250, 7200]

class TakeOffDefault(object):
    TAKEOFF_PATROL_POINT = Fighter_Near_South_Ship_Pos
    TAKEOFF_PATROL_PARAMS = [270, 20000, 20000, 250, 7200]

    A2A_PATROL_HEIGHT = 8000
    A2A_PATROL_PARAMS = [270, 20000, 20000, 250, 7200]

    A2G_TAKEOFF_AREAHUNT_PARAMS = [270, 20000, 20000, 250]
    A2G_AREAHUNT_PARAMS = [270, 20000, 20000]

    AWACS_PATROL_PARAMS = [270, 20000, 20000, 220, 7200, 2]


class RulePlayerTraining(Agent):

    def __init__(self, name, config='blue', **kwargs):
        super().__init__(name, config)
        self.state = 0
        self.side = 'blue'
        self.rl_flag = False

    # def _take_off(self, raw_obs):
    #     cmds = []
    #     fly_types = [UnitType.A2A]
    #     for type_ in fly_types:
    #         if self._get_waiting_aircraft_num(raw_obs, type_):
    #             cmds.append(
    #                 Command.takeoff_areapatrol(
    #                     BLUE_AIRPORT_ID, 1, type_,
    #                     patrol_points=Fighter_Near_South_Ship_Pos,
    #                     patrol_params=Fighter_MassArea_Params))
    #     return cmds

    def _awacs_task(self, raw_obs):
        cmds = []
        patrol_points = [-240000, 0, 130000]
        # TODO(zhoufan): 是否应该将awacs的id缓存起来
        for unit in raw_obs['units']:
            if unit['LX'] == UnitType.AWACS:
                cmds.append(
                    Command.awacs_areapatrol(
                        unit['ID'], patrol_points))
                break
        return cmds

    def step(self, obs_blue):
        cmds = []
        self.dict_units = Dict_GetOwnUnitsData(obs_blue['units'])
        self.dict_qb = Dict_GetQBData(obs_blue['qb'])
        if self.state == 0:
            # cmds.extend(self._take_off(raw_obs))
            cmds.extend(self._takeoff_areapatrol(1, UnitType.A2A, Fighter_Near_South_Ship_Pos, Fighter_MassArea_Params))
            cmds.extend(self._awacs_task(obs_blue))
            flag, ship_ids = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.SHIP)
            #
            self.own_first_ship_id = ship_ids[0]
            self.own_second_ship_id = ship_ids[1]
            cmds.extend(self._ship_movedeploy(ship_ids[0], FIRST_SHIP_POINT))
            cmds.extend(self._ship_movedeploy(ship_ids[1], FIRST_SHIP_POINT))
            self.state = 1

        if self.state == 1:
            a2a_flag, a2a_ids = Dict_GetOwnUnitsID_ByType(self.dict_units, UnitType.A2A)

            enemy_a2a_flag, enemy_a2a_ids = Dict_GetOwnUnitsID_ByType(self.dict_qb, UnitType.A2A)
            enemy_a2g_flag, enemy_a2g_ids = Dict_GetOwnUnitsID_ByType(self.dict_qb, UnitType.A2G)
            enemy_awacs_flag, enemy_awacs_ids = Dict_GetOwnUnitsID_ByType(self.dict_qb, UnitType.AWACS)

            enemy_entity_num = 0

            if enemy_a2a_flag:
                for enemy_a2a_id in enemy_a2a_ids:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, a2a_ids[0], enemy_a2a_id, 2)

                    if dist <= 120000.0:
                        enemy_entity_num += 1

            if enemy_a2g_flag:
                for enemy_a2g_id in enemy_a2g_ids:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, a2a_ids[0], enemy_a2g_id, 2)

                    if dist <= 120000.0:
                        enemy_entity_num += 1

            if enemy_awacs_flag:
                for enemy_awacs_id in enemy_awacs_ids:
                    _, dist = Dict_CalcDistByTwoUnitID(self.dict_units, self.dict_qb, a2a_ids[0], enemy_awacs_id, 2)

                    if dist <= 120000.0:
                        enemy_entity_num += 1

            if enemy_entity_num >= 3:
                self.rl_flag = True
                self.state = 2

        print('rule agent state:', self.state)

        return cmds

    # @staticmethod
    def _ship_movedeploy(self, self_id, point):
        return [Command.ship_deploy(self_id, *point, 0, 1)]

    def _takeoff_areapatrol(self, num, lx, patrol_point, patrol_params):
        # patrol_params为5个参数
        return [EnvCmd.make_takeoff_areapatrol(BLUE_AIRPORT_ID, num, lx, *patrol_point, *patrol_params)]

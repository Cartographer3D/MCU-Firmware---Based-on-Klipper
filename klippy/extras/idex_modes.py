# Support for duplication and mirroring modes for IDEX printers
#
# Copyright (C) 2021  Fabrice Gallet <tircown@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

class DualCarriages:
    def __init__(self, printer, rail_0, rail_1, axis):
        self.printer = printer
        self.axis = axis
        self.dc = (rail_0, rail_1)
        self.printer.add_object("idex_modes", self)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
                   'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
                   desc=self.cmd_SET_DUAL_CARRIAGE_help)
    def toggle_active_dc_rail(self, index):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        kin = toolhead.get_kinematics()
        for i, dc in enumerate(self.dc):
            if i != index:
                dc.inactivate(pos)
                kin.override_rail(3, dc.rail)
            elif dc.is_active() is False:
                newpos = pos[:self.axis] + [dc.axis_position] \
                            + pos[self.axis+1:]
                dc.activate(newpos)
                kin.override_rail(self.axis, dc.rail)
                toolhead.set_position(newpos)
                kin.update_limits(self.axis, dc.rail.get_range())
    def get_status(self):
        dc0, dc1 = self.dc
        if (dc0.is_active() is True):
            mode = ['FULL_CONTROL','CARRIAGE_0']
        else:
            mode = ['FULL_CONTROL','CARRIAGE_1']
        return {
            'mode': mode,
            'axis_positions': (dc0.axis_position, dc1.axis_position)
            }
    def recover_status(self, saved_status):
        # set carriage 0 active
        if (saved_status['mode'][1] == 'CARRIAGE_0'
                    and self.dc[0].is_active() is False):
            self.toggle_active_dc_rail(0)
        # set carriage 1 active
        elif (saved_status['mode'][1] == 'CARRIAGE_1'
                    and self.dc[1].is_active() is False):
            self.toggle_active_dc_rail(1)
    cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"
    def cmd_SET_DUAL_CARRIAGE(self, gcmd):
        index = gcmd.get_int('CARRIAGE', minval=0, maxval=1)
        force_homing = gcmd.get_int('HOMING', 0, minval=0, maxval=1)
        if (not(self.dc[0].is_active() == self.dc[1].is_active() == True)
                    and self.dc[index].is_active() is False):
            self.toggle_active_dc_rail(index)
            if force_homing:
                kin = self.printer.lookup_object('toolhead').get_kinematics()
                force_home_axis = getattr(kin, 'force_home_axis', None)
                if callable(force_home_axis):
                    kin.force_home_axis(self.axis, self.dc[index].rail)
                else:
                    errormes = 'Function \'force_home_axis\' is not defined \
                                in kinematic'
                    self.printer.lookup_object('gcode').respond_info(errormes)

class DualCarriagesRail:
    ACTIVE=1
    INACTIVE=2
    def __init__(self, printer, rail, axis, active, stepper_alloc_active,
                 stepper_alloc_inactive=None):
        self.printer = printer
        self.rail = rail
        self.axis = axis
        self.status = (self.INACTIVE, self.ACTIVE)[active]
        self.stepper_alloc_active = stepper_alloc_active
        self.stepper_alloc_inactive = stepper_alloc_inactive
        self.axis_position = -1
    def _stepper_alloc(self, position, active=True):
        toolhead = self.printer.lookup_object('toolhead')
        self.axis_position = position[self.axis]
        self.rail.set_trapq(None)
        if active is True:
            self.status = self.ACTIVE
            if self.stepper_alloc_active is not None:
                self.rail.setup_itersolve(*self.stepper_alloc_active)
                self.rail.set_position(position)
                self.rail.set_trapq(toolhead.get_trapq())
        else:
            self.status = self.INACTIVE
            if self.stepper_alloc_inactive is not None:
                self.rail.setup_itersolve(*self.stepper_alloc_inactive)
                self.rail.set_position(position)
                self.rail.set_trapq(toolhead.get_trapq())
    def is_active(self):
        return self.status == self.ACTIVE
    def activate(self, position):
        self._stepper_alloc(position, active=True)
    def inactivate(self, position):
        self._stepper_alloc(position, active=False)

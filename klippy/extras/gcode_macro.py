# Add ability to define custom g-code macros
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

DEFAULT_PREFIX = 'default_parameter_'


class GCodeMacro:
    def __init__(self, config):
        self.alias = config.get_name().split()[1].upper()
        self.script = config.get('gcode')
        printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        try:
            self.gcode.register_command(
                self.alias, self.cmd, desc=self.cmd_desc)
        except self.gcode.error as e:
            raise config.error(str(e))
        self.in_script = False
        self.kwparams = {}
        try:
            kvpairs = [map(str.strip, (
                o[len(DEFAULT_PREFIX):], config.get(o, '')
            )) for o in config.get_prefix_options(DEFAULT_PREFIX)]
            self.kwparams = {str(k).upper(): v for (k, v) in kvpairs if k}
        except Exception:
            raise config.error("Unable to parse default parameter values")
    cmd_desc = "G-Code macro"
    def cmd(self, params):
        if self.in_script:
            raise self.gcode.error(
                "Macro %s called recursively" % (self.alias,))
        script = ""
        kwparams = {}
        try:
            kwparams.update(self.kwparams)
            kwparams.update(params)
            script = self.script.format(**kwparams)
        except Exception:
            msg = "Macro %s script formatting failed" % (self.alias,)
            logging.exception(msg)
            raise self.gcode.error(msg)
        self.in_script = True
        try:
            self.gcode.run_script_from_command(script)
        finally:
            self.in_script = False

def load_config_prefix(config):
    return GCodeMacro(config)

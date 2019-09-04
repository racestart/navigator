#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import time

from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
#from prompt_toolkit.contrib.completers import WordCompleter

#SQLCompleter = WordCompleter(['select', 'from', 'insert',
#                              'update', 'delete', 'drop'], ignore_case=True)


class RuntimeManager:
    def __init__(self):
        self._current_dir = ""
        self._running = False
        self._prompt = '#>'

    def __handle_command(self, cmd):
        cmds = cmd.split(' ')
        if cmds[0] == 'exit':
            self._running = False
        elif cmds[0] == 'driver':
            pass
        elif cmds[0] == 'calibration':
            pass

    def run(self):
        self._running = True
        while self._running is True:
            user_input = prompt(self._prompt,
                                history=FileHistory('history.txt'),
                                auto_suggest=AutoSuggestFromHistory())
                                #completer=SQLCompleter)
            self.__handle_command(user_input)


if __name__ == '__main__':
    rtm = RuntimeManager()
    rtm.run()

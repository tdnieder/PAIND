/*
 r * Maze.c
 *
 *  Created on: Dec 2, 2015
 *      Author: daniel
 */

#include "Platform.h"
#if PL_CONFIG_HAS_LINE_MAZE
#include "Maze.h"
#include "Turn.h"
#include "CLS1.h"
#include "LineFollow.h"
#include "Event.h"
#include "UTIL1.h"
#include "Shell.h"
#include "Reflectance.h"

#define MAZE_MIN_LINE_VAL      0x100   /* minimum value indicating a line */
static uint16_t SensorHistory[REF_NOF_SENSORS]; /* value of history while moving forward */

static void MAZE_SampleSensorHistory(void) {
	uint8_t i;
	uint16_t val[REF_NOF_SENSORS];

	REF_GetSensorValues(&val[0], REF_NOF_SENSORS);
	for (i = 0; i < REF_NOF_SENSORS; i++) {
		if (val[i] >= (MAZE_MIN_LINE_VAL)) { /* only count line values */
			if (val[i] > SensorHistory[i]) {
				SensorHistory[i] = val[i];
			}
		}
	}
}

/*!
 * \brief Called during turning, will use it to sample sensor values.
 */
static bool MAZE_SampleTurnStopFunction(void) {
	MAZE_SampleSensorHistory();
	return FALSE; /* do not stop turning */
}

REF_LineKind MAZE_HistoryLineKind(void) {
	int i, cnt, cntLeft, cntRight;

	cnt = cntLeft = cntRight = 0;
	for (i = 0; i < REF_NOF_SENSORS; i++) {
		if (SensorHistory[i] > MAZE_MIN_LINE_VAL) { /* count only line values */
			cnt++;
			if (i < REF_NOF_SENSORS / 2) {
				cntLeft++;
			} else {
				cntRight++;
			}
		}
	}
	if (cnt == 0) {
		return REF_LINE_NONE;
	} else if (cnt == REF_NOF_SENSORS) {
		return REF_LINE_FULL;
	} else if (cntLeft > cntRight) {
		return REF_LINE_LEFT;
	} else { /* must be cntRight>cntLeft */
		return REF_LINE_RIGHT;
	}
}

void MAZE_ClearSensorHistory(void) {
	int i;

	for (i = 0; i < REF_NOF_SENSORS; i++) {
		SensorHistory[i] = 0;
	}
}

#define MAZE_MAX_PATH        128 /* maximum number of turns in path */

static TURN_Kind path[MAZE_MAX_PATH]; /* recorded maze */
static uint8_t pathLength; /* number of entries in path[] */
static bool isSolved = FALSE; /* if we have solved the maze */
static uint8_t index = 0;

#if 1
static TURN_Kind RevertTurn(TURN_Kind turn) {
	if (turn == TURN_LEFT90) {
		turn = TURN_RIGHT90;
	} else if (turn == TURN_RIGHT90) {
		turn = TURN_LEFT90;
	}
	return turn;
}

/**
 * \brief Reverts the path
 */
void MAZE_RevertPath(void) {
	int i, j;
	TURN_Kind tmp;

	if (pathLength == 0) {
		return;
	}
	j = pathLength - 1;
	i = 0;
	while (i <= j) {
		tmp = path[i];
		path[i] = RevertTurn(path[j]);
		path[j] = RevertTurn(tmp);
		i++;
		j--;
	}
}
#endif

/* rule: True for left hand on the wall, false for right hand on the wall */
TURN_Kind MAZE_SelectTurn(REF_LineKind prev, REF_LineKind curr, bool rule) {
	if (prev == REF_LINE_NONE && curr == REF_LINE_NONE) { /* dead end */
		MAZE_AddPath(TURN_RIGHT180);
		return TURN_RIGHT180; /* make U turn */
	} else if (prev == REF_LINE_LEFT && curr == REF_LINE_NONE) {
		MAZE_AddPath(TURN_LEFT90);
		return TURN_LEFT90;
	} else if (prev == REF_LINE_RIGHT && curr == REF_LINE_NONE) {
		MAZE_AddPath(TURN_RIGHT90);
		return TURN_RIGHT90;
	} else if (prev == REF_LINE_FULL && curr == REF_LINE_NONE) {
		if (rule) {
			MAZE_AddPath(TURN_LEFT90);
			return TURN_LEFT90; /* left hand on the wall */
		} else {
			MAZE_AddPath(TURN_RIGHT90);
			return TURN_RIGHT90; /* right hand on the wall */
		}
	} else if (prev == REF_LINE_FULL && curr == REF_LINE_STRAIGHT) {
		if (rule) {
			MAZE_AddPath(TURN_LEFT90);
			return TURN_LEFT90;
		} else {
			MAZE_AddPath(TURN_RIGHT90);
			return TURN_RIGHT90;
		}
	} else if (prev == REF_LINE_LEFT && curr == REF_LINE_STRAIGHT) {
		if (rule) {
			MAZE_AddPath(TURN_LEFT90);
			return TURN_LEFT90;
		} else {
			MAZE_AddPath(TURN_STRAIGHT);
			return TURN_STRAIGHT;
		}
	} else if (prev == REF_LINE_RIGHT && curr == REF_LINE_STRAIGHT) {
		if (rule) {
			MAZE_AddPath(TURN_STRAIGHT);
			return TURN_STRAIGHT;
		} else {
			MAZE_AddPath(TURN_RIGHT90);
			return TURN_RIGHT90;
		}
	} else if (prev == REF_LINE_FULL && curr == REF_LINE_FULL) {
		return TURN_FINISHED;
	}
	return TURN_STOP; /* error case */
}

void MAZE_SetSolved(void) {
	MAZE_SimplifyPath();
	MAZE_RevertPath();
	isSolved = TRUE;
}

bool MAZE_IsSolved(void) {
	return isSolved;
}

void MAZE_AddPath(TURN_Kind kind) {
	if (pathLength < MAZE_MAX_PATH) {
		path[pathLength] = kind;
		pathLength++;
	} else {
		/* error! */
	}
}

/*!
 *
 */
void MAZE_SimplifyPath(void) {
	int k = 0;
	int counter = 0;
	int i = 0;
	TURN_Kind pathnew[MAZE_MAX_PATH];
	do {
		counter = 0;
		k = 0;
		for (i = 0; i < pathLength; i++) {

			if (path[i] == TURN_LEFT180) {
				counter++;
				if (path[i - 1] == TURN_STRAIGHT
						&& path[i + 1] == TURN_LEFT90) {
					path[k - 1] = TURN_RIGHT90;
					i++;
				} else if (path[i - 1] == TURN_LEFT90
						&& path[i + 1] == TURN_LEFT90) {
					path[k - 1] = TURN_STRAIGHT;
					i++;
				} else if (path[i - 1] == TURN_LEFT90
						&& path[i + 1] == TURN_RIGHT90) {
					path[k - 1] = TURN_LEFT180;
					i++;
				} else if (path[i - 1] == TURN_RIGHT90
						&& path[i + 1] == TURN_LEFT90) {
					path[k - 1] = TURN_LEFT180;
					i++;
				} else if (path[i - 1] == TURN_LEFT90
						&& path[i + 1] == TURN_STRAIGHT) {
					path[k - 1] = TURN_RIGHT90;
					i++;
				} else if (path[i - 1] == TURN_RIGHT90
						&& path[i + 1] == TURN_STRAIGHT) {
					path[k - 1] = TURN_LEFT90;
					i++;
				} else if (path[i - 1] == TURN_RIGHT90
						&& path[i + 1] == TURN_RIGHT90) {
					path[k - 1] = TURN_STRAIGHT;
					i++;
				} else if (path[i - 1] == TURN_STRAIGHT
						&& path[i + 1] == TURN_RIGHT90) {
					path[k - 1] = TURN_LEFT90;
					i++;
				} else if (path[i - 1] == TURN_STRAIGHT
						&& path[i + 1] == TURN_STRAIGHT) {
					path[k - 1] = TURN_LEFT180;
					i++;
				}
				i++;
				break;

			} else {
				k++;
			}
		}
		for (i; i < pathLength; i++) {
			path[k] = path[i];
			k++;
		}
		pathLength = k;
	} while (counter != 0);
}

/*!
 * \brief Performs a turn.
 * \return Returns TRUE while turn is still in progress.
 */
uint8_t MAZE_EvaluteTurn(bool *finished, bool rule) {
	REF_LineKind historyLineKind, currLineKind;
	TURN_Kind turn;
	if (MAZE_IsSolved()) {
		if (!FollowSegment()) {
			if (index < pathLength) {
				//TURN_Turn(TURN_STEP_LINE_FW_POST_LINE,MAZE_SampleTurnStopFunction);
				TURN_Turn(TURN_STEP_LINE_FW_POST_LINE, NULL);
				TURN_Turn(MAZE_GetSolvedTurn(&index), NULL);
			} else {
				TURN_Turn(TURN_STEP_LINE_FW_POST_LINE, NULL);
				LF_StopFollowing();
				index = 0;
			}
		}
		return ERR_OK;
	} else {

		*finished = FALSE;
		currLineKind = REF_GetLineKind();
		if (currLineKind == REF_LINE_NONE) { /* nothing, must be dead end */
			MAZE_AddPath(TURN_LEFT180);
			turn = TURN_LEFT180;
		} else {
			MAZE_ClearSensorHistory(); /* clear history values */
			MAZE_SampleSensorHistory(); /* store current values */
			TURN_Turn(TURN_STEP_LINE_FW_POST_LINE, MAZE_SampleTurnStopFunction); /* do the line and beyond in one step */
			historyLineKind = MAZE_HistoryLineKind(); /* new read new values */
			currLineKind = REF_GetLineKind();
			turn = MAZE_SelectTurn(historyLineKind, currLineKind, rule);
		}
		if (turn == TURN_FINISHED) {
			*finished = TRUE;
			//LF_StopFollowing();
			return ERR_OK;
		} else if (turn == TURN_STRAIGHT) {
			//SHELL_SendString((unsigned char*) "going straight\r\n");
			return ERR_OK;
		} else if (turn == TURN_STOP) { /* should not happen here? */
			LF_StopFollowing();
			return ERR_FAILED; /* error case */
		} else { /* turn or do something */
			TURN_Turn(turn, NULL);
			return ERR_OK; /* turn finished */
		}
	}
}

static void MAZE_PrintHelp(const CLS1_StdIOType *io) {
	CLS1_SendHelpStr((unsigned char*) "maze",
			(unsigned char*) "Group of maze following commands\r\n",
			io->stdOut);
	CLS1_SendHelpStr((unsigned char*) "  help|status",
			(unsigned char*) "Shows maze help or status\r\n", io->stdOut);
	CLS1_SendHelpStr((unsigned char*) "  clear",
			(unsigned char*) "Clear the maze solution\r\n", io->stdOut);
}

#if PL_CONFIG_HAS_SHELL
static void MAZE_PrintStatus(const CLS1_StdIOType *io) {
	int i;

	CLS1_SendStatusStr((unsigned char*) "maze", (unsigned char*) "\r\n",
			io->stdOut);
	CLS1_SendStatusStr((unsigned char*) "  solved",
			MAZE_IsSolved() ?
					(unsigned char*) "yes\r\n" : (unsigned char*) "no\r\n",
			io->stdOut);
	CLS1_SendStatusStr((unsigned char*) "  path", (unsigned char*) "(",
			io->stdOut);
	CLS1_SendNum8u(pathLength, io->stdOut);
	CLS1_SendStr((unsigned char*) ") ", io->stdOut);
	for (i = 0; i < pathLength; i++) {
		CLS1_SendStr(TURN_TurnKindStr(path[i]), io->stdOut);
		CLS1_SendStr((unsigned char*) " ", io->stdOut);
	}
	CLS1_SendStr((unsigned char*) "\r\n", io->stdOut);
}

uint8_t MAZE_ParseCommand(const unsigned char *cmd, bool *handled,
		const CLS1_StdIOType *io) {
	uint8_t res = ERR_OK;

	if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP) == 0
			|| UTIL1_strcmp((char*)cmd, (char*)"maze help") == 0) {
		MAZE_PrintHelp(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS) == 0
			|| UTIL1_strcmp((char*)cmd, (char*)"maze status") == 0) {
		MAZE_PrintStatus(io);
		*handled = TRUE;
	} else if (UTIL1_strcmp((char*)cmd, (char*)"maze clear") == 0) {
		MAZE_ClearSolution();
		*handled = TRUE;
	}
	return res;
}
#endif

TURN_Kind MAZE_GetSolvedTurn(uint8_t *solvedIdx) {
	if (*solvedIdx < pathLength) {
		return path[(*solvedIdx)++];
	} else {
		return TURN_STOP;
	}
}

void MAZE_ClearSolution(void) {
	isSolved = FALSE;
	pathLength = 0;
}

void MAZE_Deinit(void) {
}

void MAZE_Init(void) {
	MAZE_ClearSolution();
}
#endif /* PL_HAS_LINE_SENSOR */

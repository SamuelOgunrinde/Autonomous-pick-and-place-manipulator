/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include "qpc.h"
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Active objects
typedef struct {
	QActive super;
} RobotController;

typedef struct {
	QActive super;
	QTimeEvt servoTimer;
} ServoController;

typedef struct {
	QActive super;
	QTimeEvt pollTimer;
} IRSensor;

// Servo movement event
typedef struct {
	QEvt super;
	uint16_t angle[4];
} ServoMoveEvt;

// Prototype declarations
void sendMove(RobotController *me, const uint16_t move[4]);
void RobotController_ctor(RobotController * const me);
void ServoController_ctor(ServoController * const me);
void IRSensor_ctor(IRSensor * const me);
QState Robot_initial(RobotController * const me, void const * const par);
QState ServoController_initial(ServoController * const me, void const * const par);
QState IRSensor_initial(IRSensor * const me, void const * const par);
QState Robot_idle(RobotController * const me, QEvt const * const e);
QState Robot_move_to_pick(RobotController * const me, QEvt const * const e);
QState Robot_grip(RobotController * const me, QEvt const * const e);
QState Robot_lift(RobotController * const me, QEvt const * const e);
QState Robot_move_to_place(RobotController * const me, QEvt const * const e);
QState Robot_release(RobotController * const me, QEvt const * const e);
QState Robot_return_home(RobotController * const me, QEvt const * const e);
QState ServoController_idle(ServoController * const me, QEvt const * const e);
QState IRSensor_monitor(IRSensor * const me, QEvt const * const e);

// Predefined servo movements
const uint16_t MOVE_HOME[4] = {0, 180, 0, 0};
const uint16_t MOVE_PICK[4] = {0, 90, 0, 0};
const uint16_t MOVE_GRIP[4] = {0, 150, 0, 0};
const uint16_t MOVE_LIFT[4] = {0, 90, 0, 0};
const uint16_t MOVE_PLACE[4] = {0, 120, 0, 0};
const uint16_t MOVE_RELEASE[4] = {0, 180, 0, 0};

void sendMove(RobotController *me, const uint16_t move[4]) {
	// Allocate an event object from an event pool
	ServoMoveEvt *evt = Q_NEW(ServoMoveEvt, MOVE_SERVO_SIG);

	for (int i = 0; i < 4; i++) {
		evt->angle[i] = move[i];
	}

	// Post desired movement (event) to the Servo Active Object queue
	QACTIVE_POST(AO_ServoController, &evt->super, me);
}

// Active object constructors
void RobotController_ctor(RobotController * const me) {
	// initialize the Active object
	QActive_ctor(&me->super, Q_STATE_CAST(&Robot_initial));
}

void ServoController_ctor(ServoController * const me) {
	// initialize the Active object
    QActive_ctor(&me->super, Q_STATE_CAST(&ServoController_initial));
    // construct the time event used for motion completion
    QTimeEvt_ctorX(&me->servoTimer, &me->super, SERVO_DONE_SIG, 0U);
}

void IRSensor_ctor(IRSensor * const me) {
	// initialize the Active object
    QActive_ctor(&me->super, Q_STATE_CAST(&IRSensor_initial));
    // construct the time event used for object detection
    QTimeEvt_ctorX(&me->pollTimer, &me->super, POLL_SIG, 0U);
}

// Active Object Initial states
QState Robot_initial(RobotController * const me, void const * const par) {
    return Q_TRAN(&Robot_idle);
}

QState ServoController_initial(ServoController * const me, void const * const par) {
    Servo_Init();
    return Q_TRAN(&ServoController_idle);
}

QState IRSensor_initial(IRSensor * const me, void const * const par) {
    IR_Init();
    return Q_TRAN(&IRSensor_monitor);
}


QState Robot_idle(RobotController * const me, QEvt const * const e) {
	QState status_;
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			status_ = Q_HANDLED();
			break;
		}
		case OBJECT_DETECTED_SIG: {
			BSP_toggleLed();
			status_ = Q_TRAN(&Robot_move_to_pick);
			break;
		}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}

QState Robot_move_to_pick(RobotController * const me, QEvt const * const e) {
	QState status_;
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			sendMove(me, MOVE_PICK);
			status_ = Q_HANDLED();
			break;
		}
		case SERVO_DONE_SIG: {
			status_ = Q_TRAN(&Robot_grip);
			break;
		}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}

QState Robot_grip(RobotController * const me, QEvt const * const e) {
	QState status_;
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			sendMove(me, MOVE_GRIP);
			status_ = Q_HANDLED();
			break;
		}
		case SERVO_DONE_SIG: {
			status_ = Q_TRAN(&Robot_lift);
			break;
		}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}

QState Robot_lift(RobotController * const me, QEvt const * const e) {
	QState status_;
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			sendMove(me, MOVE_LIFT);
			status_ = Q_HANDLED();
			break;
		}
		case SERVO_DONE_SIG: {
			status_ = Q_TRAN(&Robot_move_to_place);
			break;
		}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}

QState Robot_move_to_place(RobotController * const me, QEvt const * const e) {
	QState status_;
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			sendMove(me, MOVE_PLACE);
			status_ = Q_HANDLED();
			break;
		}
		case SERVO_DONE_SIG: {
			status_ = Q_TRAN(&Robot_release);
			break;
		}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}

QState Robot_release(RobotController * const me, QEvt const * const e) {
	QState status_;
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			sendMove(me, MOVE_RELEASE);
			status_ = Q_HANDLED();
			break;
		}
		case SERVO_DONE_SIG: {
			status_ = Q_TRAN(&Robot_return_home);
			break;
		}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}

QState Robot_return_home(RobotController * const me, QEvt const * const e) {
	QState status_;
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			sendMove(me, MOVE_HOME);
			status_ = Q_HANDLED();
			break;
		}
		case SERVO_DONE_SIG: {
			status_ = Q_TRAN(&Robot_idle);
			break;
		}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}

QState ServoController_idle(ServoController * const me, QEvt const * const e) {
	QState status_;
	switch (e->sig) {
        case MOVE_SERVO_SIG: {
            ServoMoveEvt const *evt = (ServoMoveEvt const *)e;
            for (uint8_t i = 0; i < 4; i++) {
                Servo_SetAngle(i, evt->angle[i]);
            }
            Servo_Update();
            QTimeEvt_armX(&me->servoTimer, BSP_TICKS_PER_SEC/2, 0U);
            status_ = Q_HANDLED();
            break;
        }
        case SERVO_DONE_SIG: {
        	QACTIVE_POST(AO_RobotController, Q_NEW(QEvt, SERVO_DONE_SIG), me);
        	status_ = Q_HANDLED();
        	break;
        }
        default: {
        	status_ = Q_SUPER(&QHsm_top);
        	break;
        }
	}
    return status_;
}

QState IRSensor_monitor(IRSensor * const me, QEvt const * const e) {
	QState status_;
	switch (e->sig) {
		case Q_ENTRY_SIG: {
			// sample sensor every 20 ms
			QTimeEvt_armX(&me->pollTimer, BSP_TICKS_PER_SEC/10, BSP_TICKS_PER_SEC/50);
			status_ = Q_HANDLED();
			break;
		}
		case POLL_SIG: {
		if (IR_Detected()) {
			/* send detection event to robot */
			QACTIVE_POST(AO_RobotController, Q_NEW(QEvt, OBJECT_DETECTED_SIG),
					me);
		}
		status_ = Q_HANDLED();
		break;
	}
		default: {
			status_ = Q_SUPER(&QHsm_top);
			break;
		}
	}
	return status_;
}


static QEvt const *RobotController_queue[10]; // Memory buffer for the private event queue of the active object
static RobotController robotcontroller; // Instance of the active object
QActive *AO_RobotController = &robotcontroller.super; // Pointer to the Active Object Super class

static QEvt const *ServoController_queue[10]; // Memory buffer for the private event queue of the active object
static ServoController servocontroller; // Instance of the active object
QActive *AO_ServoController = &servocontroller.super; // Pointer to the Active Object Super class

static QEvt const *IRSensor_queue[10]; // Memory buffer for the private event queue of the active object
static IRSensor irsensor; // Instance of the active object
QActive *AO_IRSensor = &irsensor.super; // Pointer to the Active Object Super class

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  BSP_Init();
  QF_init();
  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */

  // event pool
  // small events
  static QF_MPOOL_EL(QEvt) smallEvtPool[20];

  // servo movement events
  static QF_MPOOL_EL(ServoMoveEvt) servoEvtPool[10];

  QF_poolInit(smallEvtPool,
               sizeof(smallEvtPool),
               sizeof(smallEvtPool[0]));

  QF_poolInit(servoEvtPool,
              sizeof(servoEvtPool),
              sizeof(servoEvtPool[0]));

  // create AO and start it
  RobotController_ctor(&robotcontroller);
  ServoController_ctor(&servocontroller);
  IRSensor_ctor(&irsensor);

  QACTIVE_START(AO_RobotController,
		  1U,
		  RobotController_queue,
		  sizeof(RobotController_queue)/sizeof(RobotController_queue[0]),
		  (void *)0, 0U,
		  (void *)0);
  QACTIVE_START(AO_ServoController,
  		  2U,
  		  ServoController_queue,
  		  sizeof(ServoController_queue)/sizeof(ServoController_queue[0]),
  		  (void *)0, 0U,
  		  (void *)0);
  QACTIVE_START(AO_IRSensor,
  		  3U,
  		  IRSensor_queue,
  		  sizeof(IRSensor_queue)/sizeof(IRSensor_queue[0]),
  		  (void *)0, 0U,
  		  (void *)0);

  return QF_run();
  /* USER CODE END 2 */
 }


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

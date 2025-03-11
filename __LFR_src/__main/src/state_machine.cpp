typedef enum {
  IDLE,
  STRAIGHT,
  TURNING,
  SLOW_ZONE,
  OBSTACLE,
} bot_state_type


volatile bot_state_type bot_state = IDLE;

void control_system(){
  while (1){
    switch (bot_state){
      
      case IDLE:
      if (1) { // PB pressed 
        bot_state = STRAIGHT;
      }
        break;

      case STRAIGHT:
        break;

      case TURNING:
        break;

      case SLOW_ZONE:
        break;

      case OBSTACLE:
        break;
        
      default:
        break;
    }
  }
}
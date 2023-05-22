import time

from dozer_prototype.scripts.roboclaw.roboclaw_3 import Roboclaw

if __name__ == "__main__":
    
    address = 0x80
    roboclaw = Roboclaw("/dev/ttyS0", 38400)
    roboclaw.Open()

    
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)

    roboclaw.BackwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    
    # turn right
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    # turn left
    roboclaw.BackwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    """
    # square
    for _ in range(4):
      roboclaw.ForwardM1(address, 100);  # Cmd 0
      roboclaw.ForwardM2(address, 100);  # Cmd 5
      time.sleep(3)
  
      # stop motors
      roboclaw.ForwardM1(address, 0);
      roboclaw.ForwardM2(address, 0);
      time.sleep(2)
      
      # turn left
      roboclaw.ForwardM2(address, 100);  # Cmd 0
      roboclaw.BackwardM1(address, 100);  # Cmd 5
      time.sleep(2.8)
      
      # stop motors
      roboclaw.ForwardM1(address, 0);
      roboclaw.ForwardM2(address, 0);
      time.sleep(2)
    """
    
    """
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(3)
  
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.BackwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
      
    # turn left
    roboclaw.ForwardM2(address, 100);  # Cmd 0
    roboclaw.BackwardM1(address, 100);  # Cmd 5
    time.sleep(2.8)
      
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(3)
  
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.BackwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    # turn right
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(1.5)
      
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(3)
    
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    """
    
    """
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(3)
  
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
      
    # turn left
    roboclaw.ForwardM2(address, 100);  # Cmd 0
    roboclaw.BackwardM1(address, 100);  # Cmd 5
    time.sleep(2.8)
      
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.BackwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    # turn right
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(1.5)
      
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(3)
  
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.BackwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(2)

    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    # turn right
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.BackwardM2(address, 100);  # Cmd 5
    time.sleep(1.5)
      
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    
    roboclaw.ForwardM1(address, 100);  # Cmd 0
    roboclaw.ForwardM2(address, 100);  # Cmd 5
    time.sleep(3)
    
    # stop motors
    roboclaw.ForwardM1(address, 0);
    roboclaw.ForwardM2(address, 0);
    time.sleep(2)
    """

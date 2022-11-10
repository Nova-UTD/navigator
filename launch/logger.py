import os
import datetime
import logging

log = logging.getLogger("navigator") # Set name of logger

def setup_logs() -> None:
    """
    Sets up logs and log file
    """
    current_directory = os.getcwd() # Get current working directory
    final_directory = os.path.join(current_directory, r'nav_logs') # Create path to nav_logs folder
    
    if not os.path.exists(final_directory): # If /nav_logs doesn't exists
        os.makedirs(final_directory) # Create a new directory in CWD called /nav_logs

    formatted_date = datetime.date.today().strftime("%m-%d-%y_%H-%M-%S")
    log_file_name = f"navigator-log_{formatted_date}.log"
    logging.basicConfig( # Set config of logger
        filename=os.path.join(final_directory, log_file_name),
        #encoding='utf-8',
        level=logging.INFO
    )
    print(f"Logging initialized, logging to /nav_logs/{log_file_name}")
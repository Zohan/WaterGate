<?php
	/*
		by Lourdes Morales
		Execute the following command on the terminal 
		to run the python file that is checking the 
		sensors and read its output to the terminal:
		if not zero - it's a leak
			turn off water
			and
			tell the page to change the status
	*/
	while(true)
	{
		//$sensor_status = exec("sudo python check_sensors.py");
		
		$sensor_status  = exec("sudo cat status.txt");
		if ($sensor_status > 0)
		{
			exec("sudo python turn_water_off.py");
			echo $sensor_status;
			break;
		}
	}
	
?>
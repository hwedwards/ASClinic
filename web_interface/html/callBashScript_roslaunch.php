<?php
	// GET THE BASH SCRIPT NAME
	$scriptname = $_GET['scriptname'];

	// ONLY EXECUTE "SCRIPT NAMES" WITH AN EXACT MATCH
	//
	// > For the MASTER
	if ($scriptname == "sensor_systems")
	{
		// GET THE VALUES OF THE EMULATE FLAG
		// > This will be a string
		$flag_value = strtolower( $_GET['flag'] );
		// Check that the new setpoint values are numerical
		if (in_array($flag_value, array("true", "1", "yes"), true))
		{
			$flag_value = "true";
		}
		elseif (in_array($flag_value, array("false", "0", "no"), true))
		{
			$flag_value = "false";
		}
		else
		{
			echo "flag value = $flag_value, is not a boolean value.";
			exit();
		}
		// Call the bash script for launching the master
		$output = shell_exec("./bashscripts/launchRosSensorSystems.sh $flag_value");
	}
	else
	{
		$output = "launch name = $scriptname is not a valid option";
	}

	echo "$output";
?>

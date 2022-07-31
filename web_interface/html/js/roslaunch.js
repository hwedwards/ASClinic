function roslaunch_outputLabelID_switchID_clearOtherLabels_clickOtherButtons(launchName, labelID, switchID, otherLabels, otherButtons)
{
	// Convert the "launchName" to a script name for the php script
	var scriptname_for_php = "";
	scriptname_for_php = launchName;

	// Get the booleans for emulation
	switch_value = document.getElementById(switchID).checked;

	// Set the label to be sending
	if(labelID){document.getElementById(labelID).innerHTML = "sending...";}

	// Create a variable for sending an AJAX request
	var xmlhttp = new XMLHttpRequest();
	// Add the function to be run when the response is recieved
	xmlhttp.onreadystatechange = function()
	{
		if (this.readyState == 4 && this.status == 200)
		{
			// Construct and display the appropriate information
			var base_string = "";
			if(labelID){document.getElementById(labelID).innerHTML = base_string + this.responseText;}
			// Click the "other buttons" as requested
			if (otherButtons)
			{
				if ( typeof(otherButtons) == "string" )
				{
					document.getElementById(otherButtons).click();
				}
				else if (otherButtons.constructor === Array)
				{
					for (otherButtonID of otherButtons)
					{
						if (otherButtonID)
						{
							if ( typeof(otherButtonID) == "string" )
							{
								document.getElementById(otherButtonID).click();
							}
						}
					}
				}
			}

		}
		else
		{
			// Construct and display the appropriate information
			var base_string = "";
			var display_message = getDisplayMessageForXMLHttpRequest(this);
			if (this.readyState == 4)
			{
				if(labelID){document.getElementById(labelID).innerHTML = base_string + display_message;}
			}
			else
			{
				//if(labelID){document.getElementById(labelID).innerHTML = base_string + display_message;}
			}
		}
	};
	
	// Call the php script
	xmlhttp.open("GET", "callBashScript_roslaunch.php?scriptname=" + scriptname_for_php + "&flag=" + switch_value, true);
	xmlhttp.send();
	
	//if(labelID){document.getElementById(labelID).innerHTML = "ERROR: launch name = " + launchName + " is not a valid option";}
	

	// Clear the other labels as requested
	if (otherLabels)
	{
		if ( typeof(otherLabels) == "string" )
		{
			document.getElementById(otherLabels).innerHTML = "";
		}
		else if (otherLabels.constructor === Array)
		{
			for (otherLabelID of otherLabels)
			{
				if (otherLabelID)
				{
					if ( typeof(otherLabelID) == "string" )
					{
						document.getElementById(otherLabelID).innerHTML = "";
					}
				}
			}
		}
	}
}

function sendRosMessage_outputLabelID(rosMessage, labelID)
{
	// Convert the "rosMessage" to a heading string
	var scriptname_for_php = "";
	if (rosMessage == "connect")
	{
		scriptname_for_php = scriptname_for_php + "rosConnect";
	}
	else if (rosMessage == "disconnect")
	{
		scriptname_for_php = scriptname_for_php + "rosDisconnect";
	}
	else if (rosMessage == "takeoff")
	{
		scriptname_for_php = scriptname_for_php + "rosTakeoff";
	}
	else if (rosMessage == "land")
	{
		scriptname_for_php = scriptname_for_php + "rosLand";
	}
	else if (rosMessage == "motorsoff")
	{
		scriptname_for_php = scriptname_for_php + "rosMotorsoff";
	}
	else if (rosMessage == "enabledefault")
	{
		scriptname_for_php = scriptname_for_php + "rosEnabledefault";
	}
	else if (rosMessage == "enablestudent")
	{
		scriptname_for_php = scriptname_for_php + "rosEnablestudent";
	}
	else if (rosMessage == "loadyamldefault")
	{
		scriptname_for_php = scriptname_for_php + "rosLoadYamlDefault";
	}
	else if (rosMessage == "loadyamlstudent")
	{
		scriptname_for_php = scriptname_for_php + "rosLoadYamlStudent";
	}



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
			//if(labelID){document.getElementById(labelID).innerHTML = base_string + "sent";}

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
	xmlhttp.open("GET", "callBashScript.php?scriptname=" + scriptname_for_php, true);
	xmlhttp.send();
}
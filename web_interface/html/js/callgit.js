function callGit_outputLabelID_shouldAppend_shouldConfirm(gitCommand, labelID, shouldAppend, shouldConfirm)
{
	// Convert the "gitCommand" to a heding string
	var heading_string = "<p class=\"terminal-highligh-paragraph\">";
	var scriptname_for_php = "";
	var confirm_alert_text = "";
	if (gitCommand == "status")
	{
		heading_string = heading_string + "GIT STATUS";
		scriptname_for_php = scriptname_for_php + "gitStatus";
		confirm_alert_text = confirm_alert_text + "This will check the status of the repository.";
	}
	else if (gitCommand == "pull")
	{
		heading_string = heading_string + "GIT PULL";
		scriptname_for_php = scriptname_for_php + "gitPull";
		confirm_alert_text = confirm_alert_text + "This will pull the latest changes, and may fail if there are any conflicts.";
	}
	else if (gitCommand == "diff")
	{
		heading_string = heading_string + "GIT DIFF";
		scriptname_for_php = scriptname_for_php + "gitDiff";
		confirm_alert_text = confirm_alert_text + "This will show the difference between the current status and the previous commit.";
	}
	else if (gitCommand == "checkout")
	{
		heading_string = heading_string + "GIT CHECKOUT .";
		scriptname_for_php = scriptname_for_php + "gitCheckout";
		confirm_alert_text = confirm_alert_text + "This remove any changes and those changes can NOT be recovered.";
	}
	else if (gitCommand == "checkoutall")
	{
		heading_string = heading_string + "GIT CHECKOUT .";
		scriptname_for_php = scriptname_for_php + "gitCheckoutAll";
		confirm_alert_text = confirm_alert_text + "This remove any changes and those changes can NOT be recovered.";
	}
	else if (gitCommand == "checkoutmaster")
	{
		heading_string = heading_string + "GIT CHECKOUT MASTER";
		scriptname_for_php = scriptname_for_php + "gitCheckoutMaster";
		confirm_alert_text = confirm_alert_text + "This switches branch and the web interface does not offer the functionality to switch back.";
	}
	else if (gitCommand == "catkin_make")
	{
		heading_string = heading_string + "CATKIN_MAKE";
		scriptname_for_php = scriptname_for_php + "catkin_make";
		confirm_alert_text = confirm_alert_text + "This compiles the dfall-system code.";
	}
	// Add the end paragraph tag to the heading string
	heading_string = heading_string + "</p>";

	// Check the "shouldAppend" input argument
	if ( typeof(shouldAppend) !== "boolean" )
	{
		shouldAppend = true;
	}

	// Check the "shouldConfirm" input argument
	if ( typeof(shouldConfirm) !== "boolean" )
	{
		shouldConfirm = true;
	}

	// If required, confirm the user wants to perform this action
	if (shouldConfirm)
	{
		var alert_response_bool = confirm(confirm_alert_text);
		if (alert_response_bool == false)
		{
			// If false then the user pressed Cancel
			return;
		}
	}
	
	// Create a variable for sending an AJAX request
	var xmlhttp = new XMLHttpRequest();
	// Add the function to be run when the response is recieved
	xmlhttp.onreadystatechange = function()
	{
		if (this.readyState == 4 && this.status == 200)
		{
			// Construct and display the appropriate information
			var base_string = "";
			if (shouldAppend)
			{
				// Take a copy of the current contents
				var current_contents = document.getElementById(labelID).innerHTML;
				// Append the new contents to the start
				document.getElementById(labelID).innerHTML = base_string + heading_string + "<br>" + this.responseText + "<br><br>" + current_contents;
			}
			else
			{
				document.getElementById(labelID).innerHTML = base_string + heading_string + "<br>" + this.responseText;
			}

		}
		else
		{
			// Construct and display the appropriate information
			var base_string = "";
			var display_message = getDisplayMessageForXMLHttpRequest(this);
			if (shouldAppend)
			{
				if (this.readyState == 4)
				{
					// Take a copy of the current contents
					var current_contents = document.getElementById(labelID).innerHTML;
					// Append the new contents to the start
					document.getElementById(labelID).innerHTML = base_string + heading_string + "<br>" + display_message + "<br><br>" + current_contents;
				}
			}
			else
			{
				if (this.readyState == 4)
				{
					document.getElementById(labelID).innerHTML = base_string + heading_string + "<br>" + display_message;
				}
				else
				{
					document.getElementById(labelID).innerHTML = base_string + display_message;
				}
			}
		}
	};
	xmlhttp.open("GET", "callBashScript.php?scriptname=" + scriptname_for_php, true);
	xmlhttp.send();
}
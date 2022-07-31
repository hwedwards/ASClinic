function callBashScript_outputLabelID_clearOtherLabels_clickOtherButtons(bashscript, labelID, otherLabels, otherButtons)
{
	// Create a variable for sending an AJAX request
	var xmlhttp = new XMLHttpRequest();
	// Add the function to be run when the response is recieved
	xmlhttp.onreadystatechange = function()
	{
		if (this.readyState == 4 && this.status == 200)
		{
			// Construct the return string
			var base_string = "";
			document.getElementById(labelID).innerHTML = base_string + this.responseText;
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
			var display_message = getDisplayMessageForXMLHttpRequest(this);
			document.getElementById(labelID).innerHTML = display_message;
		}
	};
	xmlhttp.open("GET", "callBashScript.php?scriptname=" + bashscript, true);
	xmlhttp.send();

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



function getDisplayMessageForXMLHttpRequest(xmlhttp)
{
	var base_string = "";
	if (xmlhttp.readyState != 4)
	{
		base_string = base_string + "loading...";
	}
	else
	{
		if (xmlhttp.status != 204)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": no content.";
		}
		else if (xmlhttp.status != 403)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": request forbidden.";
		}
		else if (xmlhttp.status != 404)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": page not found.";
		}
		else if (xmlhttp.status != 500)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": internal server error.";
		}
		else if (xmlhttp.status != 501)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": no implemented.";
		}
		else if (xmlhttp.status != 502)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": bd gateway.";
		}
		else if (xmlhttp.status != 503)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": service unavailable.";
		}
		else if (xmlhttp.status != 504)
		{
			base_string = base_string + "Error " + xmlhttp.status + ": gateway timeout.";
		}
		else
		{
			base_string = base_string + "Error " + xmlhttp.status;
		}
		
	}
	// Finally return the string
	return base_string;
}


function getErrorMessageForXMLHttpRequest(xmlhttp)
{
	var base_string = "";
	if (xmlhttp.readyState != 4 && xmlhttp.status != 200)
	{
		base_string = base_string + "readyState = " + xmlhttp.readyState + " and status = " + xmlhttp.status;
	}
	else if (xmlhttp.readyState != 4)
	{
		base_string = base_string + "status is good but readyState = " + xmlhttp.readyState;
	}
	else if (xmlhttp.status != 200)
	{
		base_string = base_string + "ready state is good but status = " + xmlhttp.status;
	}
	// Finally return the string
	return base_string;
}

/*
The "XMLHttpRequest" variable has the following:
> CALLBACK FUNCTION: onreadystatechange
  Defines a function to be called when the readyState property changes

> PROPERTY: readyState
  Holds the status of the XMLHttpRequest.
  0: request not initialized
  1: server connection established
  2: request received
  3: processing request
  4: request finished and response is ready

> PROPERTY status
  200: "OK"
  403: "Forbidden"
  404: "Page not found"
  For a complete list go to the Http Messages Reference

> PROPERTY statusText
  Returns the status-text (e.g. "OK" or "Not Found")

> PROPERTY responseText
  Returns the server response as a JavaScript string

For more details see:
https://www.w3schools.com/xml/ajax_xmlhttprequest_response.asp
https://www.w3schools.com/tags/ref_httpmessages.asp
*/

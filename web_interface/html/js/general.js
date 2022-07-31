function clearLabelWithGivenID(labelID)
{
	document.getElementById(labelID).innerHTML = "";
}

function roundToDecimalPlaces(value,decimals)
{
	return Number(Math.round(value+'e'+decimals)+'e-'+decimals);
}

function incrementInputFieldWithGivenID(inputID,multiplier)
{
	//document.getElementById(inputID).value = 2.0;

	// Get the value of the input field
	var current_value = parseFloat( document.getElementById(inputID).value );

	// Get the increment of the input field
	var increment_amount = document.getElementById(inputID).step;

	// Check that the value is a number
	if (isNaN(current_value))
	{
		current_value = 0.0;
	}

	// Compute the new value
	var new_value = roundToDecimalPlaces( current_value + multiplier * increment_amount , 3 );

	// Put the new value into the field
	document.getElementById(inputID).value = new_value;
}

function putDefaultSetpointForGivenBaseID(inputBaseID)
{
	// Construct the ID for the (x,y,z,yaw) input fields
	var inputID_forX   = inputBaseID + "X";
	var inputID_forY   = inputBaseID + "Y";
	var inputID_forZ   = inputBaseID + "Z";
	var inputID_forYaw = inputBaseID + "Yaw";
	// Set the default values
	document.getElementById(inputID_forX).value = 0.0;
	document.getElementById(inputID_forY).value = 0.0;
	document.getElementById(inputID_forZ).value = 0.4;
	document.getElementById(inputID_forYaw).value  = 0.0;
}
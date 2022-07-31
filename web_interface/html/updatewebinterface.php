<?php
	$temp = shell_exec("./bashscripts/updatewebinterface.sh");
	$output = "<pre>$temp</pre>";
	echo "$output";
?>

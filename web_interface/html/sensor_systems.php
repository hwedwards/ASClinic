<?php
	include("page_header.html");
?>


<!-- <script src="js/sse_agentStatus.js?ver=0.1"></script> -->


<div class="full-window-fixed">
</div>

<div class="max-width-full-heigth-fixed">

	<div class="top-bar-container">
		<table class="top-bar-buttons-table">
			<tr>
				<td class="top-bar-motorsoff-button-cell">
					<button
						class="button-push red motorsoff"
						id="buttonMotorsOffForAgent"
						onclick="sendSpeedMessage( 0.0 , 0.0 )"
						>
						STOP MOTORS
						<div class="div-for-button-highlight-on-touchscreen red"></div>
					</button>
				</td>
			</tr>
		</table>
	</div>


	<div class="main-tabs">
		<input name="tabs" type="radio" id="main-tab-1" checked="checked" class="main-tab-input"/>
		<label for="main-tab-1" class="main-tab-label">Workflow</label>
		<div class="main-tab-panel">
			<?php
				include("sensor_systems_tab_workflow.html");
			?>
		</div>

		<input name="tabs" type="radio" id="main-tab-2" class="main-tab-input"/>
		<label for="main-tab-2" class="main-tab-label">Compile</label>
		<div class="main-tab-panel">
			<?php
				include("sensor_systems_tab_compile.html");
			?>
		</div>

		<input name="tabs" type="radio" id="main-tab-3" class="main-tab-input"/>
		<label for="main-tab-3" class="main-tab-label">ROS</label>
		<div class="main-tab-panel">
			<?php
				include("sensor_systems_tab_ros.html");
			?>
		</div>
	</div>
</div>


<?php
	include("page_footer.html");
?>

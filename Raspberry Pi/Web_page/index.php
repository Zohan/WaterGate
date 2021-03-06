﻿<!-- Index page for WaterGate web page
by Lourdes Morales -->

<!DOCTYPE html>

<html lang="en" xmlns="http://www.w3.org/1999/xhtml">
<head>
    <meta charset="utf-8" />
    <meta http-equiv="Content-Type" content="text/html;" />
    <title>WATERGATE - PIPE STATUS REPORT</title>
    <link href='https://fonts.googleapis.com/css?family=Comfortaa:300,400' rel='stylesheet' type='text/css'>
    <link href="styles.css" rel="stylesheet" type="text/css" />
    <script src="http://code.jquery.com/jquery-latest.js"></script>
    <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
	
	<h1>WATERGATE</h1>

	<div id="pipe_status">
		<h3>
			PIPE STATUS
		</h3>
		<p id="status">
			Normal
		</p>
	</div>

	<div class="control">
		<h2>CONTROL</h2>
		<button id="turn_on">
			TURN ON WATER
		</button>
	</div>
	<div class="control">
		<button id="turn_off">
			TURN OFF WATER
		</button>
	</div>
	
	<div id="blank"></div>

	<script>

		$("#turn_on").click(function()
		{
			document.getElementById('status').innerHTML = "Normal";
			document.getElementById('status').style.color = '#9bfe83';
			$('#blank').load('turn_water_on.php');
			return false;
		});
	
		$("#turn_off").click(function()
		{
			document.getElementById('status').innerHTML = "Manual shut down";
			document.getElementById('status').style.color = '#f7e94f';
			$('#blank').load('turn_water_off.php');
			return false;
		});

		var myVar = setInterval(check_status, 1000); //1000 milliseconds = one second

		function check_status()
		{
			//alert('Working!');

			var xhr;
			if (window.XMLHttpRequest)
			{ // Mozilla, Safari, ...
				xhr = new XMLHttpRequest();
			} else if (window.ActiveXObject)
			{ // IE 8 and older
				xhr = new ActiveXObject("Microsoft.XMLHTTP");
			}

			var data = "";  
			xhr.open("POST", "check_status.php", true);   
			xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");                    
			xhr.send(data);
			xhr.onreadystatechange = check_sensors;  
			function check_sensors() 
			{  
				if (xhr.readyState == 4)
				{  
					if (xhr.status == 200) 
					{

						if(xhr.response > 0)
						{
							$('#blank').load('turn_water_off.php');
							document.getElementById('status').innerHTML = "Leak detected - Emergency shut down";
							document.getElementById('status').style.color = '#fc3030';
							// send message: "Leak detected. Turning water off."
						}
					
						else
						{
							alert("An unknown error occurred with the response. Please try again.");
						}	
					}
				} 
				else 
				{  
					alert('There was a problem with the XML request.');  
				}  
			} // eof function check_sensors
					
		} // eof function check_status

	</script>
	
</body>
</html>
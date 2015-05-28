//Medusa 3D-reconstruction
//a countdown to the start of the scanning

var sTime = new Date().getTime();
var countDown = 3; //set the countdown to start from 3

//function to update the time from 3 to 0
function updateTime()
{
	//get the time
	var cTime = new Date().getTime();
	var diff = cTime - sTime; //difference between the time now and the start time(real time)
	var seconds = countDown - Math.floor(diff/1000); //3sec minus the time that have past(floor) is the time left to scanning
	if(seconds >= 0)
	{
		$("#seconds").text(seconds);
	}
	else
	{
		$("#countdown").hide();
		window.location.replace("loadingPage.html");
		clearInterval(counter)
	}
}
updateTime(); //updates the time constant to 0
var counter = setInterval(updateTime, 500);
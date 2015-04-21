var sTime = new Date().getTime();
var countDown = 10;

function updateTime()
{
	var cTime = new Date().getTime();
	var diff = cTime - sTime;
	var seconds = countDown - Math.floor(diff/1000);
	if(seconds >= 0)
	{
		$("#seconds").text(seconds);
	}
	else if(seconds > -3 && seconds < 0)
	{
		$("#countdown").hide();
		$("#msg").show();
	}
	else
	{
		$("#countdown").hide();
		window.location.replace("mainPage.html");
		clearInterval(counter)
	}
}
updateTime();
var counter = setInterval(updateTime, 500);
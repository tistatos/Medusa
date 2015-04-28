var sTime = new Date().getTime();
var countDown = 3;

function updateTime()
{
	var cTime = new Date().getTime();
	var diff = cTime - sTime;
	var seconds = countDown - Math.floor(diff/1000);
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
updateTime();
var counter = setInterval(updateTime, 500);
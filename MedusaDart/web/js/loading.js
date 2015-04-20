$(document).ready(function()
{
	var progressbar = $('#progressbar'),
		max = progressbar.attr('max'),
		time = (1000/max)*5,
		value = progressbar.val();

	var loading = function() 
	{
		value++;
		addValue = progressbar.val(value);

		$('.prograss-value').html(value + '%');

		if(value == max)
		{
			clearInterval(animate);
			window.location.replace("mainPage.html");
		}
	};

	var animate = setInterval(function()
	{
		loading();
	},time);
});
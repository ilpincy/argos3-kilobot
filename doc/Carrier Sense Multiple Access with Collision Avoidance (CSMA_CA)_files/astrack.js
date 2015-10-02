
/******************************************************************************
save this file as astrack.js and place 
<script src="/astrack.js" type="text/javascript"></script>
at the end of your page code (after the last AdSense ad unit)

You must already have the UrchinTracker installed to use this.
 
Modified with permission from Jim Rotherford's Adsense Pepper
(http://www.digitalmediaminute.com/) 

© SeoBook.com. For updates see http://www.seobook.com/archives/001370.shtml 
You are allowed to use this but you should keep this copyright notice here

******************************************************************************/
function as_click () {
	urchinTracker ('/asclick');
}

// incredibly funky onload add-event scripting, for all browsers

		 if(typeof window.addEventListener != 'undefined')
		 {
		 	//.. gecko, safari, konqueror and standard
		 	window.addEventListener('load', adsense_init, false);
		 }
		 else if(typeof document.addEventListener != 'undefined')
		 {
		 	//.. opera 7
		 	document.addEventListener('load', adsense_init, false);
		 }
		 else if(typeof window.attachEvent != 'undefined')
		 {
		 	//.. win/ie
		 	window.attachEvent('onload', adsense_init);
		 }

		 //** remove this condition to degrade older browsers
		 else
		 {
		 	//.. mac/ie5 and anything else that gets this far

		 	//if there's an existing onload function
		 	if(typeof window.onload == 'function')
		 	{
		 		//store it
		 		var existing = onload;

		 		//add new onload handler
		 		window.onload = function()
		 		{
		 			//call existing onload function
		 			existing();

		 			//call adsense_init onload function
		 			adsense_init();
		 		};
		 	}
		 	else
		 	{
		 		//setup onload function
		 		window.onload = adsense_init;
		 	}
		 }
function adsense_init () {

	if (document.all) {  //ie

		var el = document.getElementsByTagName("iframe");
	
		for(var i = 0; i < el.length; i++) {
			if(el[i].src.indexOf('googlesyndication.com') > -1) {

				el[i].onfocus =  as_click;
			}
		}
	
	} else {   // firefox
	
		window.addEventListener('beforeunload', doPageExit, false);
		window.addEventListener('mousemove', getMouse, true);
	
	}
		
}

//for firefox
var px;
var py;

function getMouse(e) {
	px=e.pageX;
	py=e.clientY;
}

function findY(obj) {
	var y = 0;
	while (obj) {
		y += obj.offsetTop;
		obj = obj.offsetParent;
	}
	return(y);
}

function findX(obj) {
	var x = 0;
	while (obj) {
		x += obj.offsetLeft;
		obj = obj.offsetParent;
	}
	return(x);
}

function doPageExit(e) {

	ad = document.getElementsByTagName("iframe");
	for (i=0; i<ad.length; i++) {
		var adLeft = findX(ad[i]);
		var adTop = findY(ad[i]);
		var inFrameX = (px > (adLeft - 10) && px < (parseInt(adLeft) + parseInt(ad[i].width) + 15));
		var inFrameY = (py > (adTop - 10) && py < (parseInt(adTop) + parseInt(ad[i].height) + 10));
		
		if (inFrameY && inFrameX) {

			urchinTracker('/asclick');
		
		}
	}

}

//end for firefox

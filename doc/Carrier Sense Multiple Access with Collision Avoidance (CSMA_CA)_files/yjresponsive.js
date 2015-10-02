/*======================================================================*\
|| #################################################################### ||
|| # Package - Joomla Template based on YJSimpleGrid Framework          ||
|| # Copyright (C) since 2010  Youjoomla LLC. All Rights Reserved.      ||
|| # Authors - Dragan Todorovic                                         ||
|| # license - PHP files are licensed under  GNU/GPL V2                 ||
|| # license - CSS  - JS - IMAGE files  are Copyrighted material        ||
|| # bound by Proprietary License of Youjoomla LLC                      ||
|| # for more information visit http://www.youjoomla.com/license.html   ||
|| # Redistribution and  modification of this software                  ||
|| # is bounded by its licenses                                         ||
|| # websites - http://www.youjoomla.com | http://www.yjsimplegrid.com  ||
|| #################################################################### ||
\*======================================================================*/
window.addEvent('load', function () {
    resize();
});
window.addEvent('resize', function () {
    newmenu();
    first_span();
    resize();
	logo_size();
    size_menu();
	yjresponsive();
	
});
window.addEvent('domready', function () {
    newmenu();
    size_menu();
	logo_size();
	yjresponsive();
	
});
function logo_size(){
	 if(Browser.name == 'ie' && (Browser.version == 7 || Browser.version == 8)) {
      var size = document.documentElement.clientWidth;
	 }else{
	  var size = window.getScrollSize().x;
	 }
	var check_ls = $('logo').hasClass('logomm');
	if(size < 980 && check_ls == false){
		$('logo').addClass('logomm');
		var_get_ls  = $('logo').getProperty('style');
		split1 		= var_get_ls.split(';');
		 if (Browser.name == 'ie'){
			split2		= split1[0].split(':');
		 }else{
			split2		= split1[1].split(':');
		 }
		true_w			= split2[1].replace("%",""); 
		
		$('logo').setStyle('width',true_w * 10);
	 }
	 if(size > 980 && check_ls == true){
		$('logo').removeClass('logomm');
		var_get_ls  = $('logo').getProperty('style');
		split1 		= var_get_ls.split(';');
		 if (Browser.name == 'ie'){
			split2		= split1[0].split(':');
		 }else{
			split2		= split1[1].split(':');
		 }
		true_w			= split2[1].replace("px",""); 
		$('logo').setStyle('width',true_w / 10+'%');
	 }
}

function size_menu(){
	
	
	 if(Browser.name == 'ie' && (Browser.version == 7 || Browser.version == 8)) {
      var size = document.documentElement.clientWidth;
	 }else{
	  var size = window.getScrollSize().x;
	 }

	 var all_li 	= $$('ul.menunav li');
	 

	 if(all_li.length > 0){
	 	var tops 		= all_li[0].getSiblings();
	 	var tops_count = tops.length+1;
	 	var width		= tops.getWidth().sum() + all_li[0].getWidth();
	 	var menu_width = $('topmenu_holder').getCoordinates().width;
	 }
	 
	if(size < 980){
		$$('.top_menu').removeClass('showmenu');
        $$('#mmenu_holder').setStyle('display', 'block');
    } else {
        $$('#mmenu_holder').setStyle('display', 'none');
		$$('.top_menu').addClass('showmenu');
    }
	if(size > 980){
		 $$('#mmenu_holder').setStyle('display', 'none');
		 $$('.top_menu').removeClass('showmenu');
	}

	
}
function resize() {
	
	
	var check_h_g = $('topmenu_holder').getParent();
	if( check_h_g.getProperty('id') == 'yjsgheadergrid'){
		var grids = $$('#yjsg1,#yjsg2,#yjsg3,#yjsgbodytop,#yjsgbodybottom,#yjsg4,#yjsg5,#yjsg6,#yjsg7');
		var menu_right		= 0;
	}else{
		var grids = $$('#yjsg1,#yjsgheadergrid,#yjsg2,#yjsg3,#yjsgbodytop,#yjsgbodybottom,#yjsg4,#yjsg5,#yjsg6,#yjsg7');
		var menu_right		= 1;
	}
	
    var size = document.documentElement.clientWidth;
    var is_sized = $(document.body).hasClass('yjresponsive');
	
	var header_height 	= $('header').getStyle('height').toInt();
	var menu_height 	= $('horiznav').getStyle('height').toInt();
	
	if (size < 980) {
		$('logo').addClass('lfloat');
		$('topmenu_holder').addClass('dropped');
		$('header').addClass('dropped');
	}else{
		$('logo').removeClass('lfloat');
		$('topmenu_holder').removeClass('dropped');
		$('header').removeClass('dropped');
		
	}
	

	
	$$('select.yjstyled').setStyle('opacity', 0);

    grids.each(function (el) {
        var getmods = el.getElements('.yjsgxhtml');
        var numb_of_mods = getmods.length;
        var get_grid_width = el.getParent().getSize().y;
		
		
		if (size < 980 && (numb_of_mods == 3 || numb_of_mods == 5)) {
			getmods[getmods.length - 1].addClass('last_mod');
		}else{
			getmods[getmods.length - 1].removeClass('last_mod');
		}
		
		if (size < 980 && numb_of_mods == 1) {
			getmods[0].addClass('only_mod');
		}else{
			getmods[0].removeClass('only_mod');
		}		

    });
}

function newmenu() {
    var size = document.documentElement.clientWidth;
    var is_there = $(document.body).hasClass('mmhere');
	$$('select.yjstyled').setStyle('opacity', 0);
    if (size < 980 && is_there == false) {
        $(document.body).addClass('mmhere');
        var select_holder = new Element('div', {
            id: 'mmenu_holder'
        });
        var selection = new Element('select', {
			events: {
				change: function() {
					window.location.href = this.get('value');
				}
			},
            id: 'mmenu',
            'class': 'yjstyled'
        });
	   $('topmenu_holder').grab(select_holder, 'top');
        select_holder.adopt(selection);
        $$('.yjm_title').each(function (el) {
            var myparent = el.getParent().getParent();
            var getlink = myparent.get('href');
            var getcurrent = myparent.getParent().getParent().getProperty('id');
			var get_p_class = myparent.getParent().getProperty('class');
			var haschilds = myparent.getParent().getParent().getProperty('class');
			var gettext =  el.get('html');

			
			
            if (getcurrent == 'current') {
                imsel = 'selected';
            } else {
                imsel = '';
            }
            var mm_options = new Element('option', {
                value: getlink,
                text: gettext,
                selected: imsel
            });
            selection.adopt(mm_options);
        });
    }
}

function first_span() {
    var select_span = document.getElements("span#yjmm_selectid"),
        first_span = select_span.length;
     var size = document.documentElement.clientWidth;
	 if (size < 980 && first_span == 0) {
		var add_first_span = new Element('span', {
			'id': 'yjmm_selectid',
			'class': 'yjmm_select',
			'html': $$('li#current .yjm_title').get('html')
		});
		$('mmenu_holder').grab(add_first_span, 'top');
    }
}

function yjresponsive() {
 if (Browser.name == 'ie' && (Browser.version == 7 || Browser.version == 8)) {
    var size = document.documentElement.clientWidth;
    var is_sized = $(document.body).hasClass('yjresponsive');
    if (size < 980 && is_sized == false) {
        $(document.body).addClass('yjresponsive');
        $(document.body).addClass('yjrp980');
        $(document.body).removeClass('yjrp480');
    }
    if (size < 480 && is_sized == true) {
        $(document.body).removeClass('yjrp980');
        $(document.body).addClass('yjrp480');
    }
    if (size < 768 && is_sized == true) {
        $(document.body).addClass('yjrp768');
    } else if (size > 768) {
        $(document.body).removeClass('yjrp768');
    }
    if (size > 480 && is_sized == true) {
        $(document.body).addClass('yjrp980');
        $(document.body).removeClass('yjrp480');
    }
    if (size > 980 && is_sized == true) {
        $(document.body).removeClass('yjresponsive');
    }
 }
}
/*
CUSTOM FORM ELEMENTS
Created by Ryan Fait
www.ryanfait.com
Visit http://ryanfait.com/ for more information.

*/
var Custom = {
    init: function () {
        var inputs = document.getElementsByTagName("input"),
            span = Array(),
            textnode, option, active;
        for (a = 0; a < inputs.length; a++) {
            if (inputs[a].className == "yjstyled") {
                span[a] = document.createElement("span");
                span[a].className = inputs[a].type;
                inputs[a].parentNode.insertBefore(span[a], inputs[a]);
                inputs[a].onchange = Custom.clear;
            }
        }
        inputs = document.getElementsByTagName("select");
        for (a = 0; a < inputs.length; a++) {
            if (inputs[a].className == "yjstyled") {
                option = inputs[a].getElementsByTagName("option");
                active = option[0].childNodes[0].nodeValue;
                textnode = document.createTextNode(active);
                for (b = 0; b < option.length; b++) {
                    if (option[b].selected == true) {
                        textnode = document.createTextNode(option[b].childNodes[0].nodeValue);
                    }
                }
                span[a] = document.createElement("span");
                span[a].className = "yjmm_select";
                span[a].id = "yjmm_selectid" + inputs[a].name;
                span[a].appendChild(textnode);
                inputs[a].parentNode.insertBefore(span[a], inputs[a]);
                if (!inputs[a].getAttribute("disabled")) {
                    inputs[a].onchange = Custom.choose;
                } else {
                    inputs[a].previousSibling.className = inputs[a].previousSibling.className += " disabled";
                }
            }
        }
      
    },

    choose: function () {
        option = this.getElementsByTagName("option");
        for (d = 0; d < option.length; d++) {
            if (option[d].selected == true) {
                document.getElementById("select" + this.name).childNodes[0].nodeValue = option[d].childNodes[0].nodeValue;
                document.location.href = this.value;
            }
        }
    }
}
window.onload = Custom.init;
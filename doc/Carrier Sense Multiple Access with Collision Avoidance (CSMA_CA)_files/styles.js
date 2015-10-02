/*
---
name: Youjoomla Mouse Effects

description: allows mouseover/out/click/clickout effects

author: Youjoomla

license: This file is NOT licensed under any BSD or MIT style license.
         Unauthorised use, editing, distribution is not allowed.

copyright: Youjoomla LLC 2012

version: 1.0
*/

window.addEvent('load', function(){
	
	
		 $$('.yjhoverfx').each(function (el) {
		  var width = el.getParent().getSize().x/3.5;
		  var height = el.getParent().getSize().y;
		  el.setStyles({width:width,height:height,left:-width,display:'block'});
		 });
		  $$('.yjhoverlink').each(function (el) {
		  var width2 = el.getParent().getSize().x;
		  var height = el.getParent().getSize().y;
		  el.setStyles({height:height,width:width2,display:'block'});
		 });
		 $$('.youheadline_hover').addEvents({
		  mouseenter: function () {
		   $(this).getElement('.yjhoverfx').morph({left:0});
		  },
		  mouseleave: function () {
		   $(this).getElement('.yjhoverfx').morph({left:-$(this).getElement('.yjhoverfx').getSize().x}); 
		  }
		 });


	  Element.Events.outerClick = {
		  base : 'click',    
		  condition : function(event){
			  event.stopPropagation();
			  return false;
		  },
		  onAdd : function(fn){
			  this.getDocument().addEvent('click', fn);
		  },
		  onRemove : function(fn){
			  this.getDocument().removeEvent('click', fn);
		  }
	  };
	  
	  
	  var box    		= $$('.yj_inputbox');
	  var width  		= box[0].getParent().getSize().x;
	  
	  box.addEvent('click', function myfunction(event){
		  box.morph({width:290});
	  }); 
	  
	  box.addEvent('outerClick', function(){ 
		  box.morph({width:width-12});
	  });
	  
	  
	  
	 $$('.yjhoverfx2').each(function (el) {
	  var width = el.getParent().getSize().x;
	  var height = el.getParent().getSize().y;
	  el.setStyles({width:width,height:height,bottom:-height,display:'block'});
	 });
	 $$('.youheadline_hover2').addEvents({
	
	  mouseenter: function () {
		var position = ($(this).getParent().getSize().y) -($(this).getElement('.yjhoverlink2_title').getSize().y);
	   $(this).getElement('.yjhoverfx2').morph({bottom:-position,opacity:[1,0.7]});
	  },
	  mouseleave: function () {
	   $(this).getElement('.yjhoverfx2').morph({bottom:-$(this).getParent().getSize().y}); 
	  }
	 });	  
	  
	  
	  
	  
	  
	  
	  
	  
 
		  
});



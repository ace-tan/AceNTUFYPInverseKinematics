package fyp;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;

public class MethodCCD extends Method {
	private static final long serialVersionUID = 1L;	
	//CCD
	double stepScaleFactor=0.1; //set the m=10
	double epsilon = 0.01;		     

	public MethodCCD(){  //setup the variables value	
		defaultSetting();  	      		
		screenPosX=210;
		screenPosY=250;
		maxIterations=100;// maximum iterations	
        this.setBackground (Color.white);
        this.setSize(screenX,screenY);
        this.setLocation(screenPosX,screenPosY);
        this.setVisible(true);
        this.addMouseListener(this);
  	}
	
	public void condition() {
      	while (count<=maxIterations) {
      		if( (Math.sqrt(Math.abs(endPoint.subtractFrom(goal).elements[0][0]))*2 
      			+(Math.abs(endPoint.subtractFrom(goal).elements[1][0]))*2) <=1) {
      			IK2DTopPanel.tfSuccess[1].setText("Yes");
      			stop();
      			break;
      		}// end if	
      		else{
      			IK2DTopPanel.tfSuccess[1].setText("No");
      			IK2DTopPanel.tfCount[1].setText(count+""); 
        			moveToGoal_NLink(goal);	
        			if(count==maxIterations)
        				stop();
          	}
          }//end while
      }//end condition  
             
      public void moveToGoal_NLink(Matrix gl) {
    	  ++count;//increment
          ct=ListOfEquation.Calct(ct,numLinks,thetas);
          st=ListOfEquation.Calst(st,numLinks,thetas);

          for(int i=0; i<numLinks; i++) {    
          	if(i==0){
          		 jointLocations[0].elements[0][0]= lengths[0]*ct[0];
                   jointLocations[0].elements[1][0]= lengths[0]*st[0];          
          	}
          	else{
          		jointLocations[i].elements[0][0]= jointLocations[i-1].elements[0][0] + lengths[i]*ct[i];
          		jointLocations[i].elements[1][0]= jointLocations[i-1].elements[1][0] + lengths[i]*st[i];
          	}
          }

         endPoint= jointLocations[numLinks-1];// last point

         for( int boneIdx = numLinks-2; boneIdx >=0; boneIdx-- ){
				// Get the vector from the current bone to the end effector position.
				double curToEndX = endPoint.elements[0][0] - jointLocations[boneIdx].elements[0][0];
				double curToEndY = endPoint.elements[1][0] - jointLocations[boneIdx].elements[1][0];
				double curToEndMag = Math.sqrt( curToEndX*curToEndX + curToEndY*curToEndY );
				//System.out.printf("\n CurToEngMag: %f" , curToEndMag);
				// Get the vector from the current bone to the target position.
				double curToTargetX = goal.elements[0][0] - jointLocations[boneIdx].elements[0][0];
				double curToTargetY = goal.elements[1][0] - jointLocations[boneIdx].elements[1][0];
				double curToTargetMag = Math.sqrt( curToTargetX*curToTargetX + curToTargetY*curToTargetY );
				double cosRotAng, sinRotAng;
				double endTargetMag = (curToEndMag*curToTargetMag);
			//	System.out.printf("\n endTargetMag: %f" , endTargetMag);
				if( endTargetMag <= epsilon ){
					cosRotAng = 1;
					sinRotAng = 0;
				}
				else{
					cosRotAng = (curToEndX*curToTargetX + curToEndY*curToTargetY) / endTargetMag;
			//		System.out.printf("\n cosRotAng: %f" ,cosRotAng);
					sinRotAng = (curToEndX*curToTargetY - curToEndY*curToTargetX) / endTargetMag;
			//		System.out.printf("\n cosRotAng: %f" ,sinRotAng);
				}

				// Clamp the cosine into range when computing the angle (might be out of range
				// due to floating point error).
				double rotAng = Math.acos( Math.max(-1, Math.min(1,cosRotAng) ) );
				if( sinRotAng < 0.0 )
					rotAng = -rotAng;
				
				// Rotate the end effector position.
				endPoint.elements[0][0] = jointLocations[boneIdx].elements[0][0] + cosRotAng*curToEndX - sinRotAng*curToEndY;
				endPoint.elements[1][0] = jointLocations[boneIdx].elements[1][0] + sinRotAng*curToEndX + cosRotAng*curToEndY;

				// Rotate the current bone in local space (this value is output to the user)
				thetas[boneIdx+1] = ListOfEquation.SimplifyAngle( thetas[boneIdx+1] + rotAng );
			}
			
			//Root to end
			double curToEndX = endPoint.elements[0][0];
			double curToEndY = endPoint.elements[1][0];
			double curToEndMag = Math.sqrt( curToEndX*curToEndX + curToEndY*curToEndY );
			double curToTargetX = goal.elements[0][0];
			double curToTargetY = goal.elements[1][0];
			double curToTargetMag = Math.sqrt(   curToTargetX*curToTargetX+ curToTargetY*curToTargetY );
			double cosRotAng,sinRotAng;
			double endTargetMag = (curToEndMag*curToTargetMag);
			if( endTargetMag <= epsilon ){
				cosRotAng = 1;
				sinRotAng = 0;
			}
			else{
				cosRotAng = (curToEndX*curToTargetX + curToEndY*curToTargetY) / endTargetMag;
				sinRotAng = (curToEndX*curToTargetY - curToEndY*curToTargetX) / endTargetMag;
			}
			double rotAng = Math.acos( Math.max(-1, Math.min(1,cosRotAng) ) );
			if( sinRotAng < 0.0 )
				rotAng = -rotAng;
			endPoint.elements[0][0] = cosRotAng*curToEndX - sinRotAng*curToEndY;
			endPoint.elements[1][0] = sinRotAng*curToEndX + cosRotAng*curToEndY;
			thetas[0] = ListOfEquation.SimplifyAngle( thetas[0] + rotAng ); 	
		
      }  
      
    @Override
  	public void paint(Graphics g){
  		super.paint(g);   		
  		draw2D.displayString(g,"Serif",12,"CCD",5, 180);
  		x[0] =screen/2;	//default center point
	        y[0] =screen/2; //default center point
	        Color [] newColor= new Color[10];
	        newColor[0]=Color.yellow;
	        newColor[1]=Color.blue;
	        newColor[2]=Color.GRAY;
	        newColor[3]=Color.green;
	        
  		if(mouseClicked==true){		
  			draw2D.drawCircle(g,cursorX,cursorY,Color.red);//draw the target
  			condition();
  		}	  
  		
  	    if(begin==true){
  			for(int i=0; i<numLinks; i++) {
      	        x[i+1] = (int)jointLocations[i].elements[0][0]+screen/2;
      	        y[i+1] = screen/2-(int)jointLocations[i].elements[1][0];
      	    }    			
  		}
  	       		
  		for(int i=0; i<numLinks;i++){        //convert radians to degree
  			IK2DTopPanel.tfTheta[i].setText(String.format("%.0f",(ListOfEquation.convertRadtoDegree(thetas[i]))%180));//cast to string
  		}   		 
  		      
  		draw2D.displayLine(g,x,y);
  		draw2D.displayNode(g,x,y,newColor,numLinks);        
        Graphics2D g2 = (Graphics2D) g;
        Ellipse2D.Double circle2 = new Ellipse2D.Double(x[2]-20+2, y[2]-20+2, 20+20, 20+20);
        g2.setPaint(newColor[2+1]);
        g2.draw(circle2);        
  	}
}
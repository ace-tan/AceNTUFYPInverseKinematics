package fyp;
//set up the window panel
import java.awt.GridLayout;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

public class IK2Dwindow extends JFrame {
	private static final long serialVersionUID = 1L;
	public IK2Dwindow(){		
		super("IK 2D Application");// set the title  
		int screenX=1280;//
		int screenY=768;//
	    this.addWindowListener(new WindowAdapter() {
	         public void windowClosing(WindowEvent windowEvent){
	            System.exit(0);
	         }        
	    });     
	    
	    JPanel controlPanel = new JPanel();
	    controlPanel.setLayout(null);	    
	    controlPanel.add(new IK2DTopPanel());
	    controlPanel.add(new MethodPINV());
	    controlPanel.add(new MethodCCD());	    
	    controlPanel.add(new MethodSecondary());
	    controlPanel.add(new MethodDLS());
	    controlPanel.add(new MethodJT());
	    controlPanel.add(new MethodAll());

	    this.add(controlPanel);
	    this.setSize(screenX,screenY); // screen size coordinate
	    this.setLayout(new GridLayout(1, 1));
	    this.setVisible(true);
	}
	
	public static void main(String[] args){		
		 SwingUtilities.invokeLater(new Runnable() {
	         @Override
	         public void run() {
	        	 IK2Dwindow DemoApp = new IK2Dwindow();//run window panel
	         }	
		 });
	}
}
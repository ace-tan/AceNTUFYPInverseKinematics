package fyp;
// Abstract Window Toolkit
import java.awt.Desktop;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
//Swing
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import java.io.File;
//class
public class Main extends JFrame{
	private static final long serialVersionUID = 1L;
	// Image iconImage= new ImageIcon("image/NTUIcon.jpg").getImage();
    private JButton jBtn2D = new JButton("2D");
    private JButton jBtnReport = new JButton("Final Report");
    private JLabel  jLabelTitle= new JLabel("Animation Techniques:");
    private JLabel  jLabelTitle2= new JLabel("Inverse kinematics");
    
    public Main(){

        jLabelTitle.setFont(new java.awt.Font("Tahoma", 0, 12));
        jLabelTitle.setLocation(5, 5);
        jLabelTitle.setSize(200, 15);

        jLabelTitle2.setFont(new java.awt.Font("Tahoma", 0, 18));
        jLabelTitle2.setLocation(5, 25);
        jLabelTitle2.setSize(200, 15);
        
        jBtn2D.setLocation(10, 50);
        jBtn2D.setSize(110, 20);
        jBtn2D.setToolTipText("2D Application");
        jBtn2D.addMouseListener(new MouseAdapter(){
            public void mouseClicked(MouseEvent e){
                IK2Dwindow.main(null);
                System.out.println("2D is loaded");
            }//end mouse clicked
        });//end jbtn2d listener
        
        jBtnReport.setLocation(10, 80);
        jBtnReport.setSize(110, 20);
        jBtnReport.setToolTipText("Final Year Project Report");
        jBtnReport.addMouseListener(new MouseAdapter(){
            public void mouseClicked(MouseEvent e){
               	try {
                    //get the file link
                    File reportFile = new File("Report/FinalReport1.0.docx");
                    if (reportFile.exists()) {
                        if (Desktop.isDesktopSupported()) {
                            Desktop.getDesktop().open(reportFile);
                        } else {
                            System.out.println("Awt Desktop is not supported!");
                        }//end else
                     } else {
                    	 System.out.println("File is not exists!");
                     }//end else
                     System.out.println("Report file opened.");
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            System.out.println("Report is loaded");
            }//end mouse clicked
        });//end button listener
        
        this.setLayout(null);
     //   frame.setIconImage(iconImage);
        this.getContentPane();  
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        //to appear center //frame.setLocationRelativeTo(null);
        this.setSize(180, 150); //canvas size
        this.setTitle("FYP");
        this.setVisible(true);
        this.setResizable(false);
        this.add(jBtn2D);
        this.add(jBtnReport);
        this.add(jLabelTitle);
        this.add(jLabelTitle2);
    }//end create gui
    	
    //main args run
    public static void main( String[] args ) {
        System.out.println("Main is loaded");
        new Main();
    }//end main args

}// end class
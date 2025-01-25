import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

public class RobotArmSim extends JPanel {
    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    // Robot kolunun uzunlukları
    private final double L1 = 100; // İlk kol uzunluğu
    private final double L2 = 100; // İkinci kol uzunluğu

    // Eklem koordinatları
    private double joint1X, joint1Y;
    private double joint2X, joint2Y;
    private double endEffectorX, endEffectorY;

    // Şu anki açı değerleri
    private double currentTheta1 = 0;
    private double currentTheta2 = 0;

    // Hedef açı değerleri
    private double targetTheta1;
    private double targetTheta2;

    // Hedef nokta
    private double targetX = 300;
    private double targetY = 300;

    private Timer animationTimer;

    public RobotArmAnimation() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.BLACK);

        // Mouse tıklaması ile hedef nokta güncelleme
        addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                targetX = e.getX();
                targetY = e.getY();

                // Yeni hedef açıları hesapla
                if (calculateInverseKinematics()) {
                    // Timer'ı başlat
                    animationTimer.start();
                } else {
                    System.out.println("Hedef noktaya ulaşılamıyor!");
                }
            }
        });

        // Animasyon zamanlayıcısı
        animationTimer = new Timer(16, e -> {
            // Adım adım açıları güncelle
            currentTheta1 += (targetTheta1 - currentTheta1) * 0.1;
            currentTheta2 += (targetTheta2 - currentTheta2) * 0.1;

            // Hareket tamamlandıysa durdur
            if (Math.abs(targetTheta1 - currentTheta1) < 0.01 && Math.abs(targetTheta2 - currentTheta2) < 0.01) {
                animationTimer.stop();
            }

            // Robot kolunu yeniden çiz
            calculateJointPositions();
            repaint();
        });
    }

    // Ters kinematik hesaplama
    private boolean calculateInverseKinematics() {
        double dx = targetX - WIDTH / 2.0;
        double dy = HEIGHT / 2.0 - targetY; // Y ekseni ters olduğu için çıkarıyoruz
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Erişilebilirlik kontrolü
        if (distance > L1 + L2) {
            return false; // Ulaşılamaz hedef
        }

        // Theta2 hesaplama
        double cosTheta2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        targetTheta2 = Math.acos(cosTheta2);

        // Theta1 hesaplama
        double k1 = L1 + L2 * Math.cos(targetTheta2);
        double k2 = L2 * Math.sin(targetTheta2);
        targetTheta1 = Math.atan2(dy, dx) - Math.atan2(k2, k1);

        return true;
    }

    // Eklem pozisyonlarını hesapla
    private void calculateJointPositions() {
        joint1X = WIDTH / 2.0 + L1 * Math.cos(currentTheta1);
        joint1Y = HEIGHT / 2.0 - L1 * Math.sin(currentTheta1);
        joint2X = joint1X + L2 * Math.cos(currentTheta1 + currentTheta2);
        joint2Y = joint1Y - L2 * Math.sin(currentTheta1 + currentTheta2);
        endEffectorX = joint2X;
        endEffectorY = joint2Y;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;

        // Antialiasing
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Hedef noktayı çiz
        g2.setColor(Color.RED);
        g2.fillOval((int) targetX - 5, (int) targetY - 5, 10, 10);

        // Robot kolu çiz
        g2.setColor(Color.GREEN);
        g2.setStroke(new BasicStroke(3));

        // İlk kol
        g2.drawLine(WIDTH / 2, HEIGHT / 2, (int) joint1X, (int) joint1Y);

        // İkinci kol
        g2.drawLine((int) joint1X, (int) joint1Y, (int) joint2X, (int) joint2Y);

        // Eklem noktaları
        g2.setColor(Color.YELLOW);
        g2.fillOval((int) joint1X - 5, (int) joint1Y - 5, 10, 10);
        g2.fillOval((int) joint2X - 5, (int) joint2Y - 5, 10, 10);

        // Merkez noktayı çiz
        g2.setColor(Color.WHITE);
        g2.fillOval(WIDTH / 2 - 5, HEIGHT / 2 - 5, 10, 10);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Robot Arm Animation");
        RobotArmAnimation simulation = new RobotArmAnimation();
        frame.add(simulation);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);
    }
}

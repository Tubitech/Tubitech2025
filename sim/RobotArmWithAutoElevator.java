import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

/**
 * Robot kolu simülasyonu:
 *  - Mouse tıklandığında hedef nokta belirlenir.
 *  - Otomatik olarak elevator (ilk pivotun dikey ofseti) ayarlanır.
 *  - Kol, ters kinematik ile hesaplanan açıları animasyonlu şekilde hedefe yönlendirir.
 *  - 360 derece dönme ile 0 derecenin aynı kabul edilmesi için açı farkını -π..+π aralığına çeker.
 *  - Elevator (pivot) dikey konumu belirli min-max sınırları arasında tutulur.
 *  - Pivot açıları için de min-max açı sınırları eklenmiştir.
 */
public class RobotArmWithAutoElevator extends JPanel {
    private static final int WIDTH = 800;   // Ekran genişliği
    private static final int HEIGHT = 600;  // Ekran yüksekliği

    // Robot kolunun uzunlukları
    private final double L1 = 307.68882; // İlk kol uzunluğu
    private final double L2 = Math.sqrt((-187.204*-187.204)+(-59.522*-59.522)); // İkinci kol uzunluğu

    // Elevator (ilk pivotun dikey konumu) için min-max (piksel cinsinden)
    private static final double ELEVATOR_MIN = -200.0;  // En fazla aşağı
    private static final double ELEVATOR_MAX = 130.0;   // En fazla yukarı

    // EKLEMLERİN (AÇILARIN) MİN-MAKS SINIRLARI (Radyan cinsinden)
    // Örnek olarak, ilk eklemi -90° ile +90° arası,
    // ikinci eklemi -120° ile +120° arası hareket ettirecek şekilde ayarlayabiliriz.
    private static final double THETA1_MIN = Math.toRadians(-90.0);
    private static final double THETA1_MAX = Math.toRadians(0.0);
    private static final double THETA2_MIN = Math.toRadians(-120.0);
    private static final double THETA2_MAX = Math.toRadians(120.0);

    // Eklem koordinatları (ileri kinematik sonucu bulunur)
    private double joint1X, joint1Y;
    private double joint2X, joint2Y;

    // Şu anki açı değerleri (animasyonda adım adım yaklaşan)
    private double currentTheta1 = 0;
    private double currentTheta2 = 0;

    // Hedef açı değerleri (ters kinematik sonucu)
    private double targetTheta1 = 0;
    private double targetTheta2 = 0;

    // Hedef nokta (mouse tıklamasıyla belirlenir)
    private double targetX = 300;
    private double targetY = 300;

    // Elevator'ın (pivotun dikey konumunun) mevcut ve hedef ofset değerleri
    private double currentElevatorOffset = -50.0;  // Başlangıç değeri
    private double targetElevatorOffset = 0.0;

    // Animasyon zamanlayıcısı (timer)
    private Timer animationTimer;

    public RobotArmWithAutoElevator() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.BLACK);

        // Mouse tıklaması ile hedef nokta güncelleme
        addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                targetX = e.getX();
                targetY = e.getY();
                System.out.println("Mouse clicked at: (" + targetX + ", " + targetY + ")");

                // 1) Elevator'ı otomatik ayarla
                if (!autoAdjustElevatorForTarget()) {
                    System.out.println("Uyari: Hedef yatay olarak (L1+L2) mesafesinden daha uzakta. Elevator ile duzeltilemez!");
                    return;
                }

                // 2) Ters kinematik hesapla (açıları hedefe uygun şekilde ayarla)
                if (calculateInverseKinematics()) {
                    System.out.println("Ters kinematik basarili. Animasyon basliyor...");
                    // 3) Timer'ı başlat (animasyonla hedef açı ve elevator'a yaklaş)
                    animationTimer.start();
                } else {
                    System.out.println("Ters kinematik basarisiz. Hedef noktaya ulasmak mumkun degil!");
                }
            }
        });

        // Animasyon zamanlayıcısı: 16 ms (~60 FPS) arayla çalışır
        animationTimer = new Timer(16, e -> {
            // A) Açılar için yumuşak geçiş (arkadan dolanmayı engellemek için -π..+π normalize)

            // 1) Theta1
            double diff1 = targetTheta1 - currentTheta1;
            while (diff1 > Math.PI)  diff1 -= 2 * Math.PI;
            while (diff1 < -Math.PI) diff1 += 2 * Math.PI;
            currentTheta1 += 0.1 * diff1;

            // 2) Theta2
            double diff2 = targetTheta2 - currentTheta2;
            while (diff2 > Math.PI)  diff2 -= 2 * Math.PI;
            while (diff2 < -Math.PI) diff2 += 2 * Math.PI;
            currentTheta2 += 0.1 * diff2;

            // (İsteğe bağlı) current açıları da clamp'leyebilirsiniz:
            currentTheta1 = clampAngle(currentTheta1, THETA1_MIN, THETA1_MAX);
            currentTheta2 = clampAngle(currentTheta2, THETA2_MIN, THETA2_MAX);

            // B) Elevator offset için yumuşak geçiş
            double elevDiff = targetElevatorOffset - currentElevatorOffset;
            currentElevatorOffset += 0.1 * elevDiff;

            // Elevator offset clamp
            if (currentElevatorOffset < ELEVATOR_MIN) {
                currentElevatorOffset = ELEVATOR_MIN;
            }
            if (currentElevatorOffset > ELEVATOR_MAX) {
                currentElevatorOffset = ELEVATOR_MAX;
            }

            // C) Animasyonun bitip bitmediğini kontrol et
            if (Math.abs(diff1) < 0.01 &&
                Math.abs(diff2) < 0.01 &&
                Math.abs(elevDiff) < 0.01) {
                // Animasyon tamamlandı
                animationTimer.stop();
                System.out.println("Animasyon bitti. currentTheta1=" + Math.toDegrees(currentTheta1)
                        + ", currentTheta2=" + Math.toDegrees(currentTheta2)
                        + ", elevator=" + currentElevatorOffset);
            }

            // Robot kolu eklemlerini yeniden hesapla
            calculateForwardKinematics();
            // Ekranı yeniden çiz
            repaint();
        });

        // Başlangıçta eklemler hesaplanıp çizilsin
        calculateForwardKinematics();
    }

    /**
     * Robotun pivot açılarını min-max arasında kısıtlayan yardımcı fonksiyon.
     */
    private double clampAngle(double angle, double minAngle, double maxAngle) {
        if (angle < minAngle) {
            angle = minAngle;
        }
        if (angle > maxAngle) {
            angle = maxAngle;
        }
        return angle;
    }

    /**
     * Otomatik olarak elevatorOffset'i ayarlayarak (targetElevatorOffset)
     * hedefin erişilebilir hale gelip gelmediğini kontrol eder.
     * Mümkünse true döner, mümkün değilse false döner.
     */
    private boolean autoAdjustElevatorForTarget() {
        double centerX = WIDTH / 2.0;
        double centerY = HEIGHT / 2.0;

        // Mevcut pivot (centerY - currentElevatorOffset)
        double baseY = centerY - currentElevatorOffset;

        // Hedefe olan yatay mesafe
        double dx = targetX - centerX;
        double absDx = Math.abs(dx);

        // Eğer hedef yatayda kolun toplam uzunluğundan (L1+L2) daha uzaksa, impossible
        if (absDx > (L1 + L2)) {
            System.out.println("autoAdjustElevatorForTarget: Yatayda erisilemez. absDx=" + absDx);
            return false;
        }

        // Kolun maksimum erişim mesafesi
        double maxReach = L1 + L2;

        // Şu anki dikey mesafe
        double currentDy = baseY - targetY;
        double distance = Math.sqrt(dx*dx + currentDy*currentDy);

        // Eğer zaten ulaşılabilirse (distance <= maxReach), elevator'ı değiştirmeye gerek yok
        if (distance <= maxReach) {
            System.out.println("autoAdjustElevatorForTarget: Zaten erisilebilir. Distance=" + distance);
            targetElevatorOffset = currentElevatorOffset;
            return true;
        }

        // distance > maxReach ise, pivot noktasını (baseY) şöyle ayarla:
        // dx^2 + (pivotY - targetY)^2 = (L1 + L2)^2
        double verticalPart = Math.sqrt(maxReach * maxReach - dx*dx);

        // Pivotu hedefin "üstünde" olsun (örnek mantık)
        double desiredPivotY = targetY - verticalPart;

        // Yeni elevator offset (hedef)
        double newElevatorOffset = centerY - desiredPivotY;

        // clamp işlemi (min-max)
        if (newElevatorOffset < ELEVATOR_MIN) {
            newElevatorOffset = ELEVATOR_MIN;
            System.out.println("autoAdjustElevatorForTarget: Yeni elevator min'e takildi -> " + newElevatorOffset);
        }
        if (newElevatorOffset > ELEVATOR_MAX) {
            newElevatorOffset = ELEVATOR_MAX;
            System.out.println("autoAdjustElevatorForTarget: Yeni elevator max'a takildi -> " + newElevatorOffset);
        }

        targetElevatorOffset = newElevatorOffset;

        // Son bir kontrol (teorik olarak tam maxReach'e denk gelmeli)
        double baseYCheck = centerY - newElevatorOffset;
        double newDy = baseYCheck - targetY;
        double newDistance = Math.sqrt(dx*dx + newDy*newDy);

        System.out.println("autoAdjustElevatorForTarget: Old distance=" + distance
                + ", new distance=" + newDistance
                + ", targetElev=" + targetElevatorOffset);

        return (newDistance <= maxReach + 0.0001);
    }

    /**
     * Ters kinematik hesaplama (targetTheta1, targetTheta2 değerlerini bulur).
     * @return true ise hesap başarılı, false ise hedefe ulaşılamaz.
     */
    private boolean calculateInverseKinematics() {
        double baseX = WIDTH / 2.0;
        double baseY = HEIGHT / 2.0 - targetElevatorOffset;

        double dx = targetX - baseX;
        double dy = baseY - targetY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Basit erişim kontrolü
        if (distance > (L1 + L2)) {
            System.out.println("calculateInverseKinematics: distance > L1+L2. distance=" + distance);
            return false;
        }

        // cosTheta2 = (dx^2 + dy^2 - L1^2 - L2^2) / (2 * L1 * L2)
        double cosTheta2 = (dx*dx + dy*dy - L1*L1 - L2*L2) / (2 * L1 * L2);
        // Numerik tolerans uygulayalım
        cosTheta2 = Math.max(-1.0, Math.min(1.0, cosTheta2));

        targetTheta2 = Math.acos(cosTheta2);

        double k1 = L1 + L2 * Math.cos(targetTheta2);
        double k2 = L2 * Math.sin(targetTheta2);
        targetTheta1 = Math.atan2(dy, dx) - Math.atan2(k2, k1);

        System.out.println("calculateInverseKinematics: distance=" + distance
                + ", targetTheta1=" + Math.toDegrees(targetTheta1)
                + ", targetTheta2=" + Math.toDegrees(targetTheta2));

        // -- EKLEMLERİ SINIRLA (Clamp) --
        // Hedef açıları, eklemin mekanik sınırlarını aşmasın
        double oldT1 = targetTheta1;
        double oldT2 = targetTheta2;

        targetTheta1 = clampAngle(targetTheta1, THETA1_MIN, THETA1_MAX);
        targetTheta2 = clampAngle(targetTheta2, THETA2_MIN, THETA2_MAX);

        // Eğer clamp sonrası hedef açıları çok değiştiyse, belki tam nokta ulaşılmaz...
        // Ama bu kodda, basitçe "elimizden gelen en yakın açı" ile yetineceğiz.
        if (Math.abs(oldT1 - targetTheta1) > 0.001 || Math.abs(oldT2 - targetTheta2) > 0.001) {
            System.out.println("calculateInverseKinematics: Aci siniri nedeniyle clamp uygulandi.");
        }

        return true;
    }

    /**
     * İleri kinematik: currentTheta1, currentTheta2 ve currentElevatorOffset
     * değerlerinden eklem (joint) koordinatları hesaplanır.
     */
    private void calculateForwardKinematics() {
        double baseX = WIDTH / 2.0;
        double baseY = HEIGHT / 2.0 - currentElevatorOffset;

        // Birinci eklemin (joint1) konumu
        joint1X = baseX + L1 * Math.cos(currentTheta1);
        joint1Y = baseY - L1 * Math.sin(currentTheta1);

        // İkinci eklemin (joint2) konumu
        joint2X = joint1X + L2 * Math.cos(currentTheta1 + currentTheta2);
        joint2Y = joint1Y - L2 * Math.sin(currentTheta1 + currentTheta2);
    }

    /**
     * Robot kolunu ve hedef noktayı ekrana çizer.
     */
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;

        // Antialiasing (daha yumuşak çizimler)
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Hedef noktayı çiz (kırmızı daire)
        g2.setColor(Color.RED);
        g2.fillOval((int) targetX - 5, (int) targetY - 5, 10, 10);

        // Robot kolu çiz
        g2.setColor(Color.GREEN);
        g2.setStroke(new BasicStroke(3));

        // Pivot noktası (baseX, baseY)
        double baseX = WIDTH / 2.0;
        double baseY = HEIGHT / 2.0 - currentElevatorOffset;

        // İlk kol (base -> joint1)
        g2.drawLine((int) baseX, (int) baseY, (int) joint1X, (int) joint1Y);

        // İkinci kol (joint1 -> joint2)
        g2.drawLine((int) joint1X, (int) joint1Y, (int) joint2X, (int) joint2Y);

        // Eklem noktaları (sarı daireler)
        g2.setColor(Color.YELLOW);
        g2.fillOval((int) joint1X - 5, (int) joint1Y - 5, 10, 10);
        g2.fillOval((int) joint2X - 5, (int) joint2Y - 5, 10, 10);

        // Pivot noktayı çiz (beyaz daire)
        g2.setColor(Color.WHITE);
        g2.fillOval((int) baseX - 5, (int) baseY - 5, 10, 10);

        // Ekrana bilgi yaz
        g2.setColor(Color.CYAN);
        g2.drawString(String.format("Elevator Offset: %.2f (min=%.1f, max=%.1f)",
                currentElevatorOffset, ELEVATOR_MIN, ELEVATOR_MAX), 10, 20);
        g2.drawString(String.format("currentTheta1=%.2f°, currentTheta2=%.2f°",
                Math.toDegrees(currentTheta1), Math.toDegrees(currentTheta2)), 10, 40);
        g2.drawString(String.format("Theta1 range=%.0f°..%.0f°, Theta2 range=%.0f°..%.0f°",
                Math.toDegrees(THETA1_MIN), Math.toDegrees(THETA1_MAX),
                Math.toDegrees(THETA2_MIN), Math.toDegrees(THETA2_MAX)), 10, 60);
    }

    /**
     * Ana giriş noktası
     */
    public static void main(String[] args) {
        JFrame frame = new JFrame("Robot Arm with Auto-Elevator & Pivot Angle Limits");
        RobotArmWithAutoElevator simulation = new RobotArmWithAutoElevator();
        frame.add(simulation);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);

        System.out.println("RobotArmWithAutoElevator baslatildi. L1=" + simulation.L1 + ", L2=" + simulation.L2);
    }
}

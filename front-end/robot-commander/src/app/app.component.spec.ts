import { ComponentFixture, TestBed } from '@angular/core/testing';
import { AppComponent } from './app.component';
import { SocketClientService } from './services/socket-client/socket-client.service';
import { RouterTestingModule } from '@angular/router/testing';
import { MatMenuModule } from '@angular/material/menu';
import { Subject } from 'rxjs';
import { MatToolbar } from '@angular/material/toolbar';
import { MatIcon } from '@angular/material/icon';
import { MatSidenavModule } from '@angular/material/sidenav';
import { NoopAnimationsModule } from '@angular/platform-browser/animations';
import { MAX_VOLTAGE, MIN_VOLTAGE } from './utils/constants';
import { InfoService } from './services/info/info.service';
import { MatSnackBar } from '@angular/material/snack-bar';

// Créer un faux service
class MockSocketClientService {
  public OnConnectionChanged = new Subject<boolean>();
  public OnLoginSuccess = new Subject<void>();
  public OnLoginError = new Subject<string>();
  connectWithUsername = jasmine.createSpy('connectWithUsername');
  disconnect = jasmine.createSpy('disconnect');
}

class MockInfoService {
    robotInfos = new Map<string, any>();
    // Simulez un observable pour robotInfos si nécessaire
  }

  const mockSnackBar = {
      open: jasmine.createSpy('open')
    };

describe('AppComponent', () => {
  let component: AppComponent;
  let fixture: ComponentFixture<AppComponent>;
  let mockSocketService: MockSocketClientService;
  let mockInfoService: MockInfoService;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [AppComponent],
      imports: [
        RouterTestingModule,
        MatMenuModule,
        MatToolbar,
        MatIcon,
        MatSidenavModule,
        NoopAnimationsModule
      ],
      providers: [
        { provide: SocketClientService, useClass: MockSocketClientService },
        { provide: InfoService, useClass: MockInfoService },
        { provide: MatSnackBar, useValue: mockSnackBar }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(AppComponent);
    localStorage.removeItem('username');
    component = fixture.componentInstance;
    mockSocketService = (TestBed.inject(
      SocketClientService
    ) as unknown) as MockSocketClientService;
    mockInfoService = TestBed.inject(InfoService) as MockInfoService;
    fixture.detectChanges();
  });

  afterEach(() => {
    localStorage.removeItem('username');
  });

  it('should connect with username if username exists in localStorage', () => {
    const testUsername = 'testUser';
    localStorage.setItem('username', testUsername);
    component.ngOnInit();
    expect(mockSocketService.connectWithUsername).toHaveBeenCalledWith(
      testUsername
    );
  });

  it('should not attempt to connect if no username in localStorage', () => {
    component.ngOnInit();
    expect(mockSocketService.connectWithUsername).not.toHaveBeenCalled();
  });

  it('should show the correct connection status icon', () => {
    mockSocketService.OnConnectionChanged.next(true);
    fixture.detectChanges();
    // Vérifiez que l'icône de connexion est correcte pour l'état connecté
    expect(component.connectionStatusIcon).toBe('link');

    mockSocketService.OnConnectionChanged.next(false);
    fixture.detectChanges();
    // Vérifiez que l'icône de connexion change correctement pour l'état déconnecté
    expect(component.connectionStatusIcon).toBe('link_off');
  });

  it('should correctly convert voltage to percentage', () => {
    // Assurez-vous que la conversion est correcte pour différents niveaux de tension
    const lowVoltage = component.convertVoltageToPercentage(MIN_VOLTAGE);
    const highVoltage = component.convertVoltageToPercentage(MAX_VOLTAGE);
    expect(lowVoltage).toBe(0);
    expect(highVoltage).toBe(100);
  });

  it('should calculate the correct average battery percentage', () => {
    // Configurez mockInfoService pour fournir des données de test
    mockInfoService.robotInfos.set('robot1', { battery_voltage: MIN_VOLTAGE });
    mockInfoService.robotInfos.set('robot2', { battery_voltage: MAX_VOLTAGE });
    // Forcez la mise à jour des données dans la composante
    component.ngOnInit();
    fixture.detectChanges();
    // Le pourcentage moyen devrait être la moyenne des pourcentages de tension min et max
    expect(component.averageBatteryPercentage()).toBe(50);
  });

  it('should determine if all robots are charged correctly', () => {
    // Configurez un scénario où tous les robots sont chargés
    mockInfoService.robotInfos.set('robot1', { battery_voltage: 12 });
    mockInfoService.robotInfos.set('robot2', { battery_voltage: 12 });
    component.ngOnInit();
    fixture.detectChanges();
    expect(component.allRobotsCharged()).toBeTruthy();

    // Configurez un scénario où au moins un robot n'est pas complètement chargé
    mockInfoService.robotInfos.set('robot2', { battery_voltage: 10 });
    component.ngOnInit(); // Simulez un nouveau cycle de vie pour la mise à jour
    fixture.detectChanges();
    expect(component.allRobotsCharged()).toBeFalsy();
  });

  it('should determine if all robots are charged correctly', () => {
    // Configurez un scénario où tous les robots sont chargés
    mockInfoService.robotInfos.set('robot1', { battery_voltage: 12 });
    mockInfoService.robotInfos.set('robot2', { battery_voltage: 12 });
    component.ngOnInit();
    fixture.detectChanges();
    expect(component.allRobotsCharged()).toBeTruthy();

    // Configurez un scénario où au moins un robot n'est pas complètement chargé
    mockInfoService.robotInfos.set('robot2', { battery_voltage: 10 });
    component.ngOnInit(); // Simulez un nouveau cycle de vie pour la mise à jour
    fixture.detectChanges();
    expect(component.allRobotsCharged()).toBeFalsy();
  });

  it('should display a success message when connected to the server', () => {
    component.connected = true;
    component.showConnectionStatus(); // Simuler une connexion réussie
    expect(mockSnackBar.open).toHaveBeenCalledWith(
      'Connecté au serveur',
      'Fermer',
      jasmine.any(Object)
    );
  });

  it('should display an error message when disconnected from the server', () => {
    component.connected = false;
    component.showConnectionStatus(); // Simuler une déconnexion
    expect(mockSnackBar.open).toHaveBeenCalledWith(
      'Déconnecté du serveur',
      'Fermer',
      jasmine.any(Object)
    );
  });

    it('should navigate to the login page when logging out', () => {
      const spy = spyOn(component['router'], 'navigate');
      component.logout();
      expect(mockSocketService.disconnect).toHaveBeenCalled();
      expect(spy).toHaveBeenCalledWith(['/login']);
    });
  });

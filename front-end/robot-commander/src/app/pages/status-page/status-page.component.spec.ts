import { ComponentFixture, TestBed } from '@angular/core/testing';
import { StatusPageComponent } from './status-page.component';
import { MatSnackBar } from '@angular/material/snack-bar';
import { SocketClientService } from 'src/app/services/socket-client/socket-client.service';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { of, Subject } from 'rxjs';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { NoopAnimationsModule } from '@angular/platform-browser/animations';
import { GridsterModule } from 'angular-gridster2';


class MockSocketClientService {
  OnConnectionChanged = new Subject<boolean>();
}

class MockMatSnackBar {
  open = jasmine.createSpy('open');
}

describe('StatusPageComponent', () => {
  let component: StatusPageComponent;
  let fixture: ComponentFixture<StatusPageComponent>;
  let mockSocketClientService: MockSocketClientService;
  let mockMatSnackBar: MockMatSnackBar;

  beforeEach(async () => {
    mockSocketClientService = new MockSocketClientService();
    mockMatSnackBar = new MockMatSnackBar();
    await TestBed.configureTestingModule({
      declarations: [StatusPageComponent],
      providers: [
        { provide: SocketClientService, useValue: mockSocketClientService },
        { provide: MatSnackBar, useValue: mockMatSnackBar }
      ],
      imports: [
        MatButtonToggleModule,
        NoopAnimationsModule,
        GridsterModule // Ajoutez GridsterModule ici
      ],
      schemas: [CUSTOM_ELEMENTS_SCHEMA]
    }).compileComponents();
  
    fixture = TestBed.createComponent(StatusPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });  

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should open success snackbar on connection', () => {
    mockSocketClientService.OnConnectionChanged.next(true);
    expect(mockMatSnackBar.open).toHaveBeenCalledWith('Connexion établie!', 'Fermer', jasmine.any(Object));
  });

  it('should open error snackbar on disconnection', () => {
    mockSocketClientService.OnConnectionChanged.next(false);
    expect(mockMatSnackBar.open).toHaveBeenCalledWith('Déconnecté', 'Fermer', jasmine.any(Object));
  });

  it('should update gridster options when lock state changes', () => {
    component.options = { draggable: { enabled: false }, resizable: { enabled: false } }; // Initialize options object
    component.updateLockState(false); // Déverrouillage
    expect(component.options?.draggable?.enabled).toBeTrue();
    expect(component.options?.resizable?.enabled).toBeTrue();
    
    component.updateLockState(true); // Verrouillage
    expect(component.options?.draggable?.enabled).toBeFalse();
    expect(component.options?.resizable?.enabled).toBeFalse();
  });

  // Ajoutez d'autres tests selon les besoins...
});

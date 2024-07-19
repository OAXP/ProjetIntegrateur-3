import { ComponentFixture, TestBed } from '@angular/core/testing';
import { SystemRuntimeComponent } from './system-runtime.component';
import { InfoService } from 'src/app/services/info/info.service';
import { of } from 'rxjs';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { MatCardModule } from '@angular/material/card';
import { RobotInfo } from 'src/app/interfaces/RobotInfo';

class MockInfoService {
  robotInfosObservable = of(
    new Map<string, RobotInfo>([
      [
        'robot1',
        {
          robot_id: 'robot1',
          status: '',
          battery_voltage: 0,
          start_time: Date.now() / 1000 - 60,
          position_x: 0,
          position_y: 0,
          init_position: { x: 0, y: 0 },
          init_orientation: { x: 0, y: 0, z: 0, w: 0 }
        }
      ] // Robot démarré il y a une minute
    ])
  );
}

describe('SystemRuntimeComponent', () => {
  let component: SystemRuntimeComponent;
  let fixture: ComponentFixture<SystemRuntimeComponent>;
  let mockInfoService: MockInfoService;

  beforeEach(async () => {
    mockInfoService = new MockInfoService();

    await TestBed.configureTestingModule({
      declarations: [SystemRuntimeComponent],
      providers: [
        { provide: InfoService, useValue: mockInfoService }
      ],
      imports: [MatCardModule],
      schemas: [CUSTOM_ELEMENTS_SCHEMA]
    }).compileComponents();

    fixture = TestBed.createComponent(SystemRuntimeComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should correctly display the runtime for robots', (done) => {
    setTimeout(() => {
      fixture.detectChanges();
      const compiled = fixture.nativeElement;
      expect(compiled.querySelector('.runtime-value').textContent).toMatch(/\d{2}:\d{2}:\d{2}/);
      done();
    }, 50); // Attendez un peu pour que le setInterval ait le temps de mettre à jour le temps
  });

  afterEach(() => {
    Object.values(component['robotTimers']).forEach(timer => clearInterval(timer));
  });
});

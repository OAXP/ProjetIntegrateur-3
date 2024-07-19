import { CommunicationService } from 'src/app/services/communication/communication.service';
import { InfoService } from 'src/app/services/info/info.service';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { of } from 'rxjs';
import { By } from '@angular/platform-browser';
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { LocateComponent } from './locate.component';
import { RobotInfo } from 'src/app/interfaces/RobotInfo';
import { ComponentFixture, TestBed } from '@angular/core/testing';

class MockInfoService {
  robotInfos = new Map<string, RobotInfo>();
  get robotInfosArray() {
    return Array.from(this.robotInfos.entries());
  }
}

class MockCommunicationService {
  identify = jasmine.createSpy('identify').and.returnValue(of({}));
}

describe('LocateComponent', () => {
  let component: LocateComponent;
  let fixture: ComponentFixture<LocateComponent>;
  let mockInfoService: MockInfoService;
  let mockCommunicationService: MockCommunicationService;

  beforeEach(async () => {
    mockInfoService = new MockInfoService();
    mockCommunicationService = new MockCommunicationService();

    await TestBed.configureTestingModule({
      declarations: [LocateComponent],
      providers: [
        { provide: InfoService, useValue: mockInfoService },
        { provide: CommunicationService, useValue: mockCommunicationService }
      ],
      imports: [MatCardModule, MatButtonModule, MatIconModule],
      schemas: [CUSTOM_ELEMENTS_SCHEMA] // Pour ignorer les éléments Material non connus
    }).compileComponents();

    fixture = TestBed.createComponent(LocateComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should call identify on CommunicationService with correct id', () => {
    const testRobotId = '001';
    component.identify(testRobotId);
    expect(mockCommunicationService.identify).toHaveBeenCalledWith(testRobotId);
  });

  it('should display no robots message when no robots connected', () => {
    mockInfoService.robotInfos = new Map();
    fixture.detectChanges();

    const noRobotsEl = fixture.debugElement.query(By.css('.no-robots'));
    expect(noRobotsEl).toBeTruthy();
  });
});


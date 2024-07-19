import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MapComponent } from './map.component';
import { InfoService } from 'src/app/services/info/info.service';
import { Router } from '@angular/router';
import { of } from 'rxjs';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';

class MockInfoService {
  mapInfoObservable = of('fake_image_data_url');
}

class MockRouter {
  navigate = jasmine.createSpy('navigate');
}

// Mock de ResizeObserver si nécessaire
class MockResizeObserver {
  observe = jasmine.createSpy('observe');
  unobserve = jasmine.createSpy('unobserve');
  disconnect = jasmine.createSpy('disconnect');
}

describe('MapComponent', () => {
  let component: MapComponent;
  let fixture: ComponentFixture<MapComponent>;
  let mockInfoService: MockInfoService;
  let mockRouter: MockRouter;

  beforeEach(async () => {
    mockInfoService = new MockInfoService();
    mockRouter = new MockRouter();

    await TestBed.configureTestingModule({
      declarations: [ MapComponent ],
      providers: [
        { provide: InfoService, useValue: mockInfoService },
        { provide: Router, useValue: mockRouter }
      ],
      schemas: [CUSTOM_ELEMENTS_SCHEMA] // Pour les éléments Angular Material
    }).compileComponents();

    fixture = TestBed.createComponent(MapComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should navigate to /map when openMap is called', () => {
    component.openMap();
    expect(mockRouter.navigate).toHaveBeenCalledWith(['/map']);
  });

  // Test pour la réception et le traitement des données d'image
  it('should draw image on canvas when receiving image data', (done) => {
    spyOn(component, 'drawImageOnCanvas').and.callThrough();

    component.ngAfterViewInit();

    fixture.whenStable().then(() => {
      expect(component.drawImageOnCanvas).toHaveBeenCalledWith('fake_image_data_url');
      done();
    });
  });

  // Ajoutez plus de tests selon les besoins...
});

import { ComponentFixture, TestBed } from '@angular/core/testing';
import { SystemInfoComponent } from './system-info.component';
import { UserManagerService } from 'src/app/services/user-manager.service';
import { of } from 'rxjs';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { MatCardModule } from '@angular/material/card';

class MockUserManagerService {
  user$ = of({ username: 'testUser', isAdmin: true });
}

describe('SystemInfoComponent', () => {
  let component: SystemInfoComponent;
  let fixture: ComponentFixture<SystemInfoComponent>;
  let mockUserManagerService: MockUserManagerService;

  beforeEach(async () => {
    mockUserManagerService = new MockUserManagerService();

    await TestBed.configureTestingModule({
      declarations: [ SystemInfoComponent ],
      providers: [
        { provide: UserManagerService, useValue: mockUserManagerService }
      ],
      imports: [MatCardModule],
      schemas: [CUSTOM_ELEMENTS_SCHEMA]
    }).compileComponents();

    fixture = TestBed.createComponent(SystemInfoComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should display user information if available', () => {
    fixture.detectChanges(); // Refresh the view to ensure data is updated
    const compiled = fixture.nativeElement;
    expect(compiled.querySelector('.info-table').textContent).toContain('testUser (admin)');
  });
});

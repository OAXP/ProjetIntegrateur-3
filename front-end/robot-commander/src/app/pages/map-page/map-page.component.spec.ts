import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MapPageComponent } from './map-page.component';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';

describe('MapPageComponent', () => {
  let component: MapPageComponent;
  let fixture: ComponentFixture<MapPageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [MapPageComponent],
      schemas: [CUSTOM_ELEMENTS_SCHEMA]
    })
    .compileComponents();

    fixture = TestBed.createComponent(MapPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  afterEach(() => {
    document.querySelector('.mat-toolbar')?.remove();
    document.querySelector('.container')?.remove();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should set container height correctly', () => {
    const toolbarHeight = 64;
    document.body.innerHTML += `
      <div class="mat-toolbar" style="height: ${toolbarHeight}px;"></div>
      <div class="container"></div>
    `;
    (component as any).setContainerHeight();

    const container = document.querySelector('.container') as HTMLElement;
    expect(container.style.height).toBe(`calc(-${toolbarHeight}px + 100vh)`);
  });
  
});

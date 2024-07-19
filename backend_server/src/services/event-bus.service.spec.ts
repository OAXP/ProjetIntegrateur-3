import { expect } from 'chai';
import { EventBus } from '@src/services/event-bus.service';

describe('EventBusService', () => {
    let eventBus: EventBus;

    beforeEach(() => {
        eventBus = new EventBus();
    });

    it('should subscribe to an event and execute the callback when event is published', () => {
        const eventName = 'testEvent';
        let callbackCalled = false;
        const callback = () => {
            callbackCalled = true;
        };

        eventBus['listeners'] = new Map<string, Function[]>([['testEvent', [callback]]]);

        eventBus.subscribe(eventName, callback);
        eventBus.publish(eventName);

        expect(callbackCalled).to.be.true;
    });

    it('should subscribe to multiple events and execute the callback for each event when published', () => {
        const eventNames = ['event1', 'event2', 'event3'];
        let callbackCalledCount = 0;
        const callback = () => {
            callbackCalledCount++;
        };

        eventNames.forEach(eventName => eventBus.subscribe(eventName, callback));
        eventNames.forEach(eventName => eventBus.publish(eventName));

        expect(callbackCalledCount).to.equal(eventNames.length);
    });

    it('should not execute callback if event is not subscribed', () => {
        const eventName = 'unsubscribedEvent';
        let callbackCalled = false;
        const callback = () => {
            callbackCalled = true;
        };

        eventBus.publish(eventName);

        expect(callbackCalled).to.be.false;
    });

    it('should pass data to the callback when event is published with data', () => {
        const eventName = 'dataEvent';
        const eventData = { key: 'value' };
        let receivedData: any = null;
        const callback = (data: any) => {
            receivedData = data;
        };

        eventBus.subscribe(eventName, callback);
        eventBus.publish(eventName, eventData);

        expect(receivedData).to.deep.equal(eventData);
    });
});

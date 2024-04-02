import { subsystemList, type SubsystemInfo } from '$lib/api';
import { asyncReadable, type Loadable } from '@square/svelte-store';

/**
 * Store for the list of subsystems.
 */
export const subsystemStore: Loadable<SubsystemInfo[]> = asyncReadable([], subsystemList);

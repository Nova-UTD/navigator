import { subsystemList, type SubsystemInfo } from '$lib/api';
import { asyncReadable, type Loadable } from '@square/svelte-store';

export const subsystemStore: Loadable<SubsystemInfo[]> = asyncReadable([], subsystemList);
